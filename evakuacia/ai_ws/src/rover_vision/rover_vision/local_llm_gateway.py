from __future__ import annotations

from dataclasses import asdict, dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


@dataclass(frozen=True)
class ModelSpec:
    id: str
    label: str
    parameters: str
    quantization: str
    filename: str
    expected_size: int
    disk_gb: float
    profile: str
    accent: str
    gpu_layers: str = "all"


MODEL_SPECS = (
    ModelSpec(
        id="qwen3.5-2b",
        label="Qwen 3.5 · 2B",
        parameters="2B",
        quantization="Q4_K_M",
        filename="qwen3.5-2b/Qwen3.5-2B-Q4_K_M.gguf",
        expected_size=1280835840,
        disk_gb=1.3,
        profile="Быстрые команды и короткие ответы",
        accent="#c9ff45",
    ),
    ModelSpec(
        id="qwen3.5-9b",
        label="Qwen 3.5 · 9B",
        parameters="9B",
        quantization="Q4_K_M",
        filename="qwen3.5-9b/Qwen3.5-9B-Q4_K_M.gguf",
        expected_size=5680522464,
        disk_gb=5.3,
        profile="Основная агентная модель",
        accent="#ffb347",
    ),
    ModelSpec(
        id="qwen3.5-35b-a3b",
        label="Qwen 3.5 · 35B-A3B",
        parameters="35B / 3B active",
        quantization="Q3_K_S",
        filename="qwen3.5-35b-a3b/Qwen_Qwen3.5-35B-A3B-Q3_K_S.gguf",
        expected_size=16404920128,
        disk_gb=16.4,
        profile="Сложные планы, максимальное качество",
        accent="#ff5f4a",
        gpu_layers="auto",
    ),
)


class GatewayError(RuntimeError):
    pass


class UnknownModelError(GatewayError):
    pass


class ModelManager:
    def __init__(
        self,
        *,
        llama_server: Path,
        models_dir: Path,
        api_key: str,
        api_key_file: Path | None = None,
        backend_host: str = "127.0.0.1",
        backend_port: int = 8092,
        device: str = "Vulkan0",
        context_size: int = 8192,
        threads: int = 8,
        startup_timeout_s: float = 360.0,
        log_dir: Path | None = None,
    ) -> None:
        self.llama_server = llama_server.expanduser().resolve()
        self.models_dir = models_dir.expanduser().resolve()
        self.api_key = api_key
        self.api_key_file = api_key_file.expanduser().resolve() if api_key_file else None
        self.backend_host = backend_host
        self.backend_port = int(backend_port)
        self.device = device
        self.context_size = int(context_size)
        self.threads = int(threads)
        self.startup_timeout_s = float(startup_timeout_s)
        self.log_dir = (log_dir or Path("~/.cache/rover-llm/logs")).expanduser()
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self._specs = {spec.id: spec for spec in MODEL_SPECS}
        self._state_lock = threading.RLock()
        self.request_lock = threading.Lock()
        self._process: subprocess.Popen[bytes] | None = None
        self._log_handle = None
        self._current_model: str | None = None
        self._requested_model: str | None = None
        self._state = "stopped"
        self._state_since = time.monotonic()
        self._last_error = ""
        self._last_log_path = ""
        self._request_count = 0
        self._last_request_ms: float | None = None
        self._last_load_ms: float | None = None

    @property
    def backend_url(self) -> str:
        return f"http://{self.backend_host}:{self.backend_port}"

    def model_spec(self, model_id: str) -> ModelSpec:
        try:
            return self._specs[str(model_id)]
        except KeyError as exc:
            raise UnknownModelError(f"Unknown model: {model_id}") from exc

    def model_path(self, spec: ModelSpec) -> Path:
        return self.models_dir / spec.filename

    def _set_state(self, state: str, *, error: str = "") -> None:
        with self._state_lock:
            self._state = state
            self._state_since = time.monotonic()
            self._last_error = error

    def status(self) -> dict[str, Any]:
        with self._state_lock:
            process = self._process
            alive = process is not None and process.poll() is None
            if process is not None and not alive and self._state in {"loading", "ready"}:
                self._state = "error"
                self._last_error = f"llama-server exited with code {process.returncode}"
            models = []
            for spec in MODEL_SPECS:
                path = self.model_path(spec)
                file_size = path.stat().st_size if path.is_file() else 0
                downloaded = (
                    file_size == spec.expected_size
                    and not Path(f"{path}.aria2").exists()
                )
                entry = asdict(spec)
                entry.update({
                    "downloaded": downloaded,
                    "file_size": file_size,
                    "active": self._current_model == spec.id and alive,
                    "loading": self._requested_model == spec.id and self._state == "loading",
                })
                models.append(entry)
            return {
                "state": self._state,
                "state_elapsed_s": round(time.monotonic() - self._state_since, 1),
                "current_model": self._current_model if alive else None,
                "requested_model": self._requested_model,
                "backend_alive": alive,
                "backend_pid": process.pid if alive else None,
                "backend_url": self.backend_url,
                "last_error": self._last_error,
                "last_log_path": self._last_log_path,
                "request_count": self._request_count,
                "last_request_ms": self._last_request_ms,
                "last_load_ms": self._last_load_ms,
                "models": models,
            }

    def _health_ready(self) -> bool:
        request = Request(
            f"{self.backend_url}/health",
            headers={"Authorization": f"Bearer {self.api_key}"},
        )
        try:
            with urlopen(request, timeout=1.5) as response:
                return response.status == 200
        except (HTTPError, URLError, TimeoutError):
            return False

    def _stop_process(self) -> None:
        process = self._process
        self._process = None
        self._current_model = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=12.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    pass
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None

    def stop(self) -> None:
        with self.request_lock:
            with self._state_lock:
                self._stop_process()
                self._requested_model = None
                self._set_state("stopped")

    def ensure_model(self, model_id: str) -> dict[str, Any]:
        spec = self.model_spec(model_id)
        path = self.model_path(spec)
        if (
            not path.is_file()
            or path.stat().st_size != spec.expected_size
            or Path(f"{path}.aria2").exists()
        ):
            raise GatewayError(f"Model file is not downloaded: {path}")

        load_started = time.monotonic()
        with self._state_lock:
            if (
                self._current_model == spec.id
                and self._process is not None
                and self._process.poll() is None
                and self._health_ready()
            ):
                return self.status()

            self._stop_process()
            self._requested_model = spec.id
            self._set_state("loading")
            log_path = self.log_dir / f"{spec.id}.log"
            self._last_log_path = str(log_path)
            self._log_handle = log_path.open("ab", buffering=0)

            command = [
                str(self.llama_server),
                "--model", str(path),
                "--alias", spec.id,
                "--host", self.backend_host,
                "--port", str(self.backend_port),
                "--device", self.device,
                "--gpu-layers", spec.gpu_layers,
                "--ctx-size", str(self.context_size),
                "--parallel", "1",
                "--threads", str(self.threads),
                "--threads-batch", str(self.threads),
                "--cache-type-k", "q8_0",
                "--cache-type-v", "q8_0",
                "--jinja",
                "--reasoning", "off",
                "--metrics",
                "--offline",
            ]
            if self.api_key_file is not None:
                command.extend(["--api-key-file", str(self.api_key_file)])
            else:
                command.extend(["--api-key", self.api_key])
            self._process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=self._log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )

        deadline = time.monotonic() + self.startup_timeout_s
        while time.monotonic() < deadline:
            process = self._process
            if process is None:
                raise GatewayError("Model process disappeared during startup")
            return_code = process.poll()
            if return_code is not None:
                error = f"llama-server exited with code {return_code}; see {self._last_log_path}"
                self._set_state("error", error=error)
                raise GatewayError(error)
            if self._health_ready():
                with self._state_lock:
                    self._current_model = spec.id
                    self._requested_model = None
                    self._last_load_ms = round((time.monotonic() - load_started) * 1000.0, 1)
                    self._set_state("ready")
                return self.status()
            time.sleep(0.35)

        self._stop_process()
        error = f"Timed out loading {spec.id}; see {self._last_log_path}"
        self._set_state("error", error=error)
        raise GatewayError(error)

    def record_request(self, elapsed_ms: float) -> None:
        with self._state_lock:
            self._request_count += 1
            self._last_request_ms = round(elapsed_ms, 1)

    def log_tail(self, max_bytes: int = 12000) -> str:
        path = Path(self._last_log_path) if self._last_log_path else None
        if path is None or not path.is_file():
            return ""
        with path.open("rb") as stream:
            size = path.stat().st_size
            stream.seek(max(0, size - max_bytes))
            return stream.read().decode("utf-8", errors="replace")


def ensure_api_key(path: Path) -> str:
    import secrets

    path = path.expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        value = path.read_text(encoding="utf-8").strip()
        if value:
            return value
    value = "sk-local-" + secrets.token_urlsafe(30)
    path.write_text(value + "\n", encoding="utf-8")
    path.chmod(0o600)
    return value


INDEX_HTML = r'''<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Model Bay · SVERK</title>
  <style>
    :root{--paper:#e8e5dc;--ink:#151813;--muted:#6b7068;--line:#a9ada4;--panel:#f4f1e8;--acid:#c9ff45;--orange:#ff9f43;--red:#ff5f4a;--black:#11130f}
    *{box-sizing:border-box}html{background:var(--black)}body{margin:0;color:var(--ink);background:var(--paper);font-family:"IBM Plex Mono","Cascadia Mono",monospace;min-height:100vh}
    body:before{content:"";position:fixed;inset:0;pointer-events:none;opacity:.22;background-image:radial-gradient(#34382f 0.7px,transparent .8px);background-size:7px 7px;mix-blend-mode:multiply}
    .shell{width:min(1480px,96vw);margin:auto;padding:22px 0 44px;position:relative}.topline{display:flex;justify-content:space-between;align-items:center;border-bottom:3px solid var(--ink);padding:0 0 14px;margin-bottom:16px}
    .brand{display:flex;align-items:center;gap:14px}.mark{width:42px;height:42px;background:var(--ink);color:var(--acid);display:grid;place-items:center;font:900 21px/1 "Impact",sans-serif;transform:rotate(-3deg)}
    .brand strong{font:900 18px/1 "Arial Narrow","Impact",sans-serif;letter-spacing:.08em}.brand small,.meta{color:var(--muted);font-size:10px;letter-spacing:.12em;text-transform:uppercase}
    .live{display:flex;align-items:center;gap:8px;font-size:11px;text-transform:uppercase}.dot{width:9px;height:9px;background:var(--orange);border-radius:50%;box-shadow:0 0 0 5px rgba(255,159,67,.2)}.dot.ready{background:#65bd3d;box-shadow:0 0 0 5px rgba(101,189,61,.2)}.dot.error{background:var(--red)}
    .hero{display:grid;grid-template-columns:1.25fr .75fr;gap:16px;margin-bottom:16px}.hero-main{background:var(--ink);color:var(--paper);padding:28px 30px;min-height:250px;position:relative;overflow:hidden}.hero-main:after{content:"03";position:absolute;right:-12px;bottom:-50px;color:#2f342b;font:900 230px/.8 "Impact",sans-serif}
    .kicker{color:var(--acid);text-transform:uppercase;font-size:11px;letter-spacing:.2em}.hero h1{position:relative;z-index:1;margin:15px 0 18px;font:900 clamp(54px,7vw,106px)/.78 "Arial Narrow","Impact",sans-serif;text-transform:uppercase;letter-spacing:-.035em}.hero p{position:relative;z-index:1;max-width:700px;color:#b9bdb4;font-size:12px;line-height:1.65;margin:0}
    .endpoint{background:var(--panel);border:2px solid var(--ink);padding:22px;display:flex;flex-direction:column;justify-content:space-between}.endpoint h2,.section-title{margin:0;font:900 28px/1 "Arial Narrow","Impact",sans-serif;text-transform:uppercase;letter-spacing:.02em}.endpoint code{display:block;background:var(--ink);color:var(--acid);padding:14px;margin:18px 0 10px;font-size:11px;word-break:break-all}.endpoint label,.field label{display:block;color:var(--muted);font-size:10px;text-transform:uppercase;letter-spacing:.12em;margin-bottom:7px}
    input,textarea,select{width:100%;border:1px solid var(--line);background:#fffdf5;color:var(--ink);padding:12px;font:12px/1.5 inherit;outline:none;border-radius:0}input:focus,textarea:focus,select:focus{border-color:var(--ink);box-shadow:3px 3px 0 var(--acid)}textarea{min-height:130px;resize:vertical}
    button{border:1px solid var(--ink);background:var(--ink);color:var(--paper);padding:12px 15px;font:800 11px/1 inherit;text-transform:uppercase;letter-spacing:.08em;cursor:pointer;transition:transform .12s,box-shadow .12s}button:hover{transform:translate(-2px,-2px);box-shadow:4px 4px 0 var(--acid)}button:disabled{opacity:.45;cursor:wait;transform:none;box-shadow:none}.ghost{background:transparent;color:var(--ink)}
    .models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-bottom:16px}.model{border:1px solid var(--line);background:var(--panel);padding:18px;position:relative;min-height:220px;display:flex;flex-direction:column}.model:before{content:"";height:7px;background:var(--accent);position:absolute;left:-1px;right:-1px;top:-1px}.model.active{border:3px solid var(--ink);padding:16px}.model .num{color:var(--muted);font-size:10px;letter-spacing:.14em;margin:4px 0 22px}.model h3{font:900 31px/.92 "Arial Narrow","Impact",sans-serif;text-transform:uppercase;margin:0 0 9px}.model p{font-size:11px;line-height:1.55;color:var(--muted);margin:0 0 16px}.chips{display:flex;gap:6px;flex-wrap:wrap;margin-top:auto;margin-bottom:14px}.chip{border:1px solid var(--line);padding:5px 7px;font-size:9px;text-transform:uppercase}.model button{width:100%;background:var(--accent);color:var(--ink)}
    .workbench{display:grid;grid-template-columns:minmax(0,1.35fr) minmax(300px,.65fr);gap:16px}.tester,.telemetry{border:2px solid var(--ink);background:var(--panel);padding:22px}.section-head{display:flex;justify-content:space-between;gap:12px;align-items:start;margin-bottom:20px}.section-head small{font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:.1em}.row{display:grid;grid-template-columns:1fr 150px 150px;gap:10px}.field{margin-bottom:12px}.actions{display:flex;gap:8px;align-items:center}.actions button:first-child{min-width:180px;background:var(--acid);color:var(--ink)}
    .answer{margin-top:16px;border-top:1px solid var(--line);padding-top:16px;min-height:130px;white-space:pre-wrap;font-size:12px;line-height:1.65}.placeholder{color:var(--muted)}
    .telemetry-grid{display:grid;grid-template-columns:1fr 1fr;border-top:1px solid var(--line);border-left:1px solid var(--line);margin-top:17px}.metric{padding:13px;border-right:1px solid var(--line);border-bottom:1px solid var(--line)}.metric span{display:block;color:var(--muted);font-size:9px;text-transform:uppercase;margin-bottom:8px}.metric strong{font:900 23px/1 "Arial Narrow","Impact",sans-serif}.log{margin-top:14px;background:var(--ink);color:#94a08f;padding:12px;height:160px;overflow:auto;white-space:pre-wrap;font-size:9px;line-height:1.45}
    .notice{margin-top:14px;border-left:6px solid var(--orange);padding:10px 12px;background:#fff7e6;font-size:10px;line-height:1.55}.pulse{animation:pulse 1s ease-in-out infinite alternate}@keyframes pulse{to{opacity:.45}}@media(max-width:980px){.hero,.workbench{grid-template-columns:1fr}.models{grid-template-columns:1fr}.row{grid-template-columns:1fr}.hero h1{font-size:64px}}
  </style>
</head>
<body><main class="shell">
  <div class="topline"><div class="brand"><div class="mark">S</div><div><strong>SVERK MODEL BAY</strong><br><small>Local inference switchboard / <span id="hostLabel"></span></small></div></div><div class="live"><i id="statusDot" class="dot"></i><span id="statusText">connecting</span></div></div>
  <section class="hero"><div class="hero-main"><div class="kicker">LLM routing station · OpenAI compatible</div><h1>Three minds.<br>One key.</h1><p>Три локальные Qwen-модели за одним API. Модель выбирается полем <b>model</b>; VLM продолжает независимо работать на 8091 и вызывается вашим сценарием после ответа LLM.</p></div>
  <aside class="endpoint"><div><h2>Общий доступ</h2><code id="endpoint"></code><label for="apiKey">Bearer API key</label><input id="apiKey" type="password" placeholder="sk-local-…" autocomplete="off"></div><button class="ghost" id="copyCurl">Скопировать curl</button></aside></section>
  <section id="models" class="models"></section>
  <section class="workbench"><div class="tester"><div class="section-head"><div><div class="meta">Test bench / 01</div><h2 class="section-title">Проверка ответа</h2></div><small>Запрос идёт через тот же /v1/chat/completions</small></div>
    <div class="row"><div class="field"><label for="modelSelect">Модель</label><select id="modelSelect"></select></div><div class="field"><label for="temperature">Temperature</label><input id="temperature" type="number" min="0" max="2" step="0.05" value="0.1"></div><div class="field"><label for="maxTokens">Max tokens</label><input id="maxTokens" type="number" min="16" max="4096" step="16" value="512"></div></div>
    <div class="field"><label for="systemPrompt">System prompt</label><input id="systemPrompt" value="Ты локальный агент мобильного робота. Отвечай кратко по-русски."></div>
    <div class="field"><label for="prompt">Запрос</label><textarea id="prompt">Составь краткий план: робот увидел препятствие в ближайшей клетке. Что он должен проверить перед объездом?</textarea></div>
    <div class="actions"><button id="run">Запустить тест</button><button id="clear" class="ghost">Очистить</button><span id="busy" class="meta"></span></div><div id="answer" class="answer"><span class="placeholder">Ответ модели появится здесь.</span></div>
  </div><aside class="telemetry"><div class="meta">Runtime / 02</div><h2 class="section-title">Телеметрия</h2><div class="telemetry-grid"><div class="metric"><span>Состояние</span><strong id="mState">—</strong></div><div class="metric"><span>Активная</span><strong id="mModel">—</strong></div><div class="metric"><span>Последняя загрузка</span><strong id="mElapsed">—</strong></div><div class="metric"><span>Последний запрос</span><strong id="mLatency">—</strong></div><div class="metric"><span>Скорость</span><strong id="mSpeed">—</strong></div><div class="metric"><span>Запросов</span><strong id="mCount">0</strong></div></div><div id="notice" class="notice">VLM не останавливается и не блокируется этим шлюзом.</div><pre id="log" class="log">backend log: waiting…</pre></aside></section>
</main><script>
const $=s=>document.querySelector(s), modelsEl=$('#models'), keyEl=$('#apiKey');
const apiOrigin=`${location.protocol}//${location.hostname}:${location.port||8082}`;$('#hostLabel').textContent=location.hostname;$('#endpoint').textContent=apiOrigin+'/v1';
keyEl.value=localStorage.getItem('sverkLlmKey')||'';keyEl.oninput=()=>localStorage.setItem('sverkLlmKey',keyEl.value.trim());let status={},lastTimings=null;
function auth(){return {'Authorization':'Bearer '+keyEl.value.trim(),'Content-Type':'application/json'}}
function esc(v){return String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function renderModels(){if(!status.models)return;modelsEl.innerHTML=status.models.map((m,i)=>`<article class="model ${m.active?'active':''}" style="--accent:${m.accent}"><div class="num">BAY / 0${i+1} · ${m.downloaded?'LOCAL':'DOWNLOADING'}</div><h3>${esc(m.label)}</h3><p>${esc(m.profile)}</p><div class="chips"><span class="chip">${esc(m.parameters)}</span><span class="chip">${esc(m.quantization)}</span><span class="chip">${m.disk_gb} GB</span></div><button data-model="${m.id}" ${m.loading||!m.downloaded?'disabled':''}>${!m.downloaded?'Докачивается…':m.active?'Активна':m.loading?'Загрузка…':'Активировать'}</button></article>`).join('');modelsEl.querySelectorAll('button').forEach(b=>b.onclick=()=>selectModel(b.dataset.model,b));const select=$('#modelSelect'),selected=select.value||status.current_model||'qwen3.5-9b';select.innerHTML=status.models.map(m=>`<option value="${m.id}" ${m.downloaded?'':'disabled'}>${esc(m.label)} · ${m.quantization}${m.downloaded?'':' · докачивается'}</option>`).join('');select.value=status.models.some(m=>m.id===selected&&m.downloaded)?selected:(status.current_model||status.models.find(m=>m.downloaded)?.id||'')}
async function selectModel(model,button){if(!keyEl.value.trim())return alert('Введите общий API key.');button.disabled=true;button.textContent='Загрузка…';try{const r=await fetch('/api/models/select',{method:'POST',headers:auth(),body:JSON.stringify({model})});const d=await r.json();if(!r.ok)throw Error(d.error||r.statusText);await poll()}catch(e){alert(e.message)}finally{button.disabled=false}}
async function poll(){try{const r=await fetch('/api/status',{cache:'no-store'});status=await r.json();const state=status.state||'unknown';$('#statusText').textContent=state;$('#statusDot').className='dot '+state;$('#mState').textContent=state;$('#mModel').textContent=status.current_model?.replace('qwen3.5-','')||status.requested_model?.replace('qwen3.5-','')||'—';$('#mElapsed').textContent=status.last_load_ms?Math.round(status.last_load_ms)+' ms':'—';$('#mLatency').textContent=status.last_request_ms?status.last_request_ms+' ms':'—';$('#mCount').textContent=status.request_count||0;$('#notice').textContent=status.last_error||'VLM не останавливается и не блокируется этим шлюзом.';renderModels();if(status.last_log_path){const lr=await fetch('/api/log?tail=5000',{headers:{'Authorization':'Bearer '+keyEl.value.trim()},cache:'no-store'});if(lr.ok)$('#log').textContent=await lr.text()}}catch(e){$('#statusText').textContent='offline';$('#statusDot').className='dot error'}setTimeout(poll,1000)}
$('#run').onclick=async()=>{const button=$('#run');if(!keyEl.value.trim())return alert('Введите общий API key.');button.disabled=true;$('#busy').textContent='model request…';$('#answer').innerHTML='<span class="placeholder pulse">Модель думает…</span>';lastTimings=null;const started=performance.now();try{const body={model:$('#modelSelect').value,messages:[{role:'system',content:$('#systemPrompt').value},{role:'user',content:$('#prompt').value}],temperature:Number($('#temperature').value),max_tokens:Number($('#maxTokens').value),stream:false};const r=await fetch('/v1/chat/completions',{method:'POST',headers:auth(),body:JSON.stringify(body)});const d=await r.json();if(!r.ok)throw Error(d.error?.message||d.error||r.statusText);const answer=d.choices?.[0]?.message?.content||JSON.stringify(d,null,2);$('#answer').textContent=answer;lastTimings=d.timings||null;const elapsed=performance.now()-started;$('#mLatency').textContent=Math.round(elapsed)+' ms';$('#mSpeed').textContent=lastTimings?.predicted_per_second?Number(lastTimings.predicted_per_second).toFixed(1)+' t/s':'—'}catch(e){$('#answer').textContent='Ошибка: '+e.message}finally{button.disabled=false;$('#busy').textContent=''}};
$('#clear').onclick=()=>{$('#answer').innerHTML='<span class="placeholder">Ответ модели появится здесь.</span>'};
$('#copyCurl').onclick=async()=>{const text='curl -k '+apiOrigin+'/v1/chat/completions -H "Authorization: Bearer '+(keyEl.value.trim()||'YOUR_KEY')+'" -H "Content-Type: application/json" -d \'{"model":"qwen3.5-9b","messages":[{"role":"user","content":"Привет"}]}\'';await navigator.clipboard.writeText(text);$('#copyCurl').textContent='Скопировано';setTimeout(()=>$('#copyCurl').textContent='Скопировать curl',1200)};
poll();
</script></body></html>'''.encode("utf-8")


def make_handler(manager: ModelManager, api_key: str):
    class GatewayHandler(BaseHTTPRequestHandler):
        server_version = "SverkLocalLLM/1.0"
        protocol_version = "HTTP/1.1"

        def _cors(self) -> None:
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")

        def _send_bytes(self, status: int, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self._cors()
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, status: int, payload: Any) -> None:
            self._send_bytes(
                status,
                json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )

        def _authorized(self) -> bool:
            header = self.headers.get("Authorization", "")
            candidate = header[7:].strip() if header.lower().startswith("bearer ") else ""
            return bool(candidate) and hmac.compare_digest(candidate, api_key)

        def _require_auth(self) -> bool:
            if self._authorized():
                return True
            self._send_json(401, {
                "error": {
                    "message": "Invalid or missing Bearer API key",
                    "type": "authentication_error",
                }
            })
            return False

        def _read_json(self, limit: int = 4 * 1024 * 1024) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > limit:
                raise ValueError("Invalid request size")
            data = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(data, dict):
                raise ValueError("JSON body must be an object")
            return data

        def do_OPTIONS(self) -> None:  # noqa: N802
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self._cors()
            self.end_headers()

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            if path in {"/", "/index.html"}:
                self._send_bytes(200, INDEX_HTML, "text/html; charset=utf-8")
                return
            if path in {"/health", "/api/status"}:
                self._send_json(200, manager.status())
                return
            if path == "/api/log":
                if not self._require_auth():
                    return
                self._send_bytes(200, manager.log_tail().encode("utf-8"), "text/plain; charset=utf-8")
                return
            if path == "/v1/models":
                if not self._require_auth():
                    return
                now = int(time.time())
                self._send_json(200, {
                    "object": "list",
                    "data": [
                        {"id": spec.id, "object": "model", "created": now, "owned_by": "sverk-local"}
                        for spec in MODEL_SPECS
                    ],
                })
                return
            self._send_json(404, {"error": "Not found"})

        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            if not self._require_auth():
                return
            try:
                payload = self._read_json()
                if path == "/api/models/select":
                    model_id = str(payload.get("model", ""))
                    with manager.request_lock:
                        result = manager.ensure_model(model_id)
                    self._send_json(200, result)
                    return
                if path == "/v1/chat/completions":
                    self._proxy_chat(payload)
                    return
                self._send_json(404, {"error": "Not found"})
            except UnknownModelError as error:
                self._send_json(400, {"error": {"message": str(error), "type": "invalid_request_error"}})
            except (GatewayError, ValueError, json.JSONDecodeError) as error:
                self._send_json(503, {"error": {"message": str(error), "type": "gateway_error"}})
            except Exception as error:  # pragma: no cover - runtime guard
                self._send_json(500, {"error": {"message": str(error), "type": "internal_error"}})

        def _proxy_chat(self, payload: dict[str, Any]) -> None:
            model_id = str(payload.get("model") or "qwen3.5-9b")
            manager.model_spec(model_id)
            payload["model"] = model_id
            encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            started = time.perf_counter()
            with manager.request_lock:
                manager.ensure_model(model_id)
                request = Request(
                    f"{manager.backend_url}/v1/chat/completions",
                    data=encoded,
                    headers={
                        "Authorization": f"Bearer {api_key}",
                        "Content-Type": "application/json; charset=utf-8",
                    },
                    method="POST",
                )
                try:
                    response = urlopen(request, timeout=600.0)
                except HTTPError as error:
                    body = error.read()
                    self._send_bytes(error.code, body, error.headers.get_content_type() or "application/json")
                    return
                except URLError as error:
                    raise GatewayError(f"Backend connection failed: {error}") from error

                content_type = response.headers.get("Content-Type", "application/json")
                content_length = response.headers.get("Content-Length")
                self.send_response(response.status)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                if content_length:
                    self.send_header("Content-Length", content_length)
                else:
                    self.send_header("Connection", "close")
                    self.close_connection = True
                self._cors()
                self.end_headers()
                while True:
                    chunk = response.read(64 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
                response.close()
            manager.record_request((time.perf_counter() - started) * 1000.0)

        def log_message(self, _format: str, *_args: Any) -> None:
            return

    return GatewayHandler


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True
