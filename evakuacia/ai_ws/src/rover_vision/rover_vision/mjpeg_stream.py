from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from threading import Condition, Thread
from typing import Callable
from urllib.parse import parse_qs, urlsplit


INDEX_HTML = '''<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Cell Gate — Rover Vision</title><style>
:root{--ink:#e9efe9;--muted:#819087;--panel:#111714;--line:#2a3730;--acid:#c7ff38;--red:#ff4d3d;--amber:#ffc247}
*{box-sizing:border-box}body{margin:0;min-height:100vh;color:var(--ink);background:#090d0b;font-family:"IBM Plex Mono","Cascadia Mono",monospace}main{width:min(1420px,96vw);margin:auto;padding:24px 0 40px}
header{display:flex;align-items:end;justify-content:space-between;margin-bottom:14px}.eyebrow{color:var(--acid);font-size:11px;letter-spacing:.2em}h1,h2{font-family:Impact,"DIN Condensed",sans-serif;text-transform:uppercase}h1{margin:5px 0 0;font-size:clamp(30px,4vw,58px);line-height:.9}h2{font-size:24px;margin:8px 0 14px}
#status{border:1px solid var(--line);background:var(--panel);padding:10px 14px;text-transform:uppercase;font-weight:700}.clear{color:var(--acid)}.blocked{color:var(--red)}.unknown,.starting,.calibrating{color:var(--amber)}
.grid{display:grid;grid-template-columns:minmax(0,1fr) 360px;gap:14px}.viewport,aside{background:var(--panel);border:1px solid var(--line)}.viewport{overflow:hidden}canvas{display:block;width:100%;height:auto;touch-action:none;cursor:crosshair}aside{padding:18px;max-height:calc(100vh - 125px);overflow:auto}
label,.small{font-size:11px;color:var(--muted)}select,button{width:100%;padding:12px;margin:7px 0;border:1px solid var(--line);font:700 12px inherit}select{background:#080c0a;color:var(--ink)}button{background:var(--acid);color:#10150f;cursor:pointer}button.secondary{background:transparent;color:var(--ink)}button.danger{background:transparent;color:var(--red);border-color:#633}
.readout{border-top:1px solid var(--line);margin-top:14px;padding-top:10px}.row{display:flex;justify-content:space-between;gap:12px;padding:5px 0;font-size:11px}.row span:first-child{color:var(--muted)}
.samples{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;margin-top:9px}.sample{border:1px solid var(--line);padding:5px}.sample img{width:100%;aspect-ratio:4/3;object-fit:cover;display:block}.sample button{padding:7px;margin:4px 0 0;font-size:10px}.poses{font-size:10px;color:#afbbb3;padding-left:18px}.hint{font-size:11px;line-height:1.5;color:#afbbb3}
@media(max-width:900px){.grid{grid-template-columns:1fr}aside{max-height:none}header{align-items:start;gap:12px;flex-direction:column}}
</style></head><body><main><header><div><div class="eyebrow">SVERK // CELL CLEARANCE</div><h1>Одна клетка впереди</h1></div><div id="status" class="starting">starting</div></header>
<section class="grid"><div class="viewport"><canvas id="view" width="800" height="640"></canvas></div><aside>
<label>Редактируемый профиль</label><select id="profile"><option value="normal">Обычная дорога</option><option value="bridge">Мост</option></select>
<div class="readout"><div class="row"><span>Активен по /rover/grid_pose</span><strong id="active">—</strong></div><div class="row"><span>Текущая клетка</span><span id="pose">нет данных</span></div><div class="row"><span>Эталонов в профиле</span><strong id="count">0</strong></div></div>
<h2>Зона контроля</h2><p class="hint">Перетащите четыре точки. Размер зоны можно менять после калибровки: модель автоматически пересоберётся из сохранённых полных кадров.</p>
<button id="saveRoi" class="secondary">Сохранить границы зоны</button><button id="add">Добавить текущий clear-кадр</button>
<h2>Кадры калибровки</h2><div id="samples" class="samples"></div>
<h2>Позиции моста</h2><p class="hint">Остановите робот в каждой из четырёх позиций (2 моста × 2 направления) и добавьте текущую позицию.</p><button id="addPose" class="secondary">Добавить текущую позицию как мост</button><ol id="poses" class="poses"></ol>
<div class="readout"><div class="row"><span>Диагностика</span><a href="/stream.mjpg" target="_blank" style="color:var(--acid)">MJPEG ↗</a></div></div>
</aside></section></main><script>
const c=document.getElementById('view'),x=c.getContext('2d'),st=document.getElementById('status'),profile=document.getElementById('profile');let roi=[.4,.68,.6,.68,.64,.9,.36,.9],drag=-1,dirty=false,bmp=null;
function draw(){if(!bmp)return;c.width=bmp.width;c.height=bmp.height;x.drawImage(bmp,0,0);let p=[];for(let i=0;i<4;i++)p.push([roi[2*i]*c.width,roi[2*i+1]*c.height]);x.beginPath();x.moveTo(...p[0]);p.slice(1).forEach(v=>x.lineTo(...v));x.closePath();x.fillStyle='rgba(199,255,56,.13)';x.fill();x.strokeStyle='#c7ff38';x.lineWidth=4;x.stroke();p.forEach((v,i)=>{x.beginPath();x.arc(v[0],v[1],13,0,7);x.fillStyle='#090d0b';x.fill();x.stroke();x.fillStyle='#c7ff38';x.font='bold 13px monospace';x.textAlign='center';x.textBaseline='middle';x.fillText(i+1,...v)})}
function pos(e){let r=c.getBoundingClientRect();return[(e.clientX-r.left)*c.width/r.width,(e.clientY-r.top)*c.height/r.height]}c.onpointerdown=e=>{let q=pos(e),best=1e9;for(let i=0;i<4;i++){let d=Math.hypot(q[0]-roi[2*i]*c.width,q[1]-roi[2*i+1]*c.height);if(d<best){best=d;drag=i}}if(best>55)drag=-1;else c.setPointerCapture(e.pointerId)};c.onpointermove=e=>{if(drag<0)return;let q=pos(e);roi[2*drag]=Math.max(0,Math.min(1,q[0]/c.width));roi[2*drag+1]=Math.max(0,Math.min(1,q[1]/c.height));dirty=true;draw()};c.onpointerup=c.onpointercancel=()=>drag=-1;
async function frames(){try{let r=await fetch('/source.jpg?t='+Date.now(),{cache:'no-store'});if(r.ok){let b=await createImageBitmap(await r.blob());if(bmp)bmp.close();bmp=b;draw()}}catch(e){}setTimeout(frames,250)}
async function api(path,opt={}){let r=await fetch(path,opt);if(!r.ok)throw new Error(await r.text());return r.headers.get('content-type')?.includes('json')?r.json():r.text()}function body(v){return{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(v)}}
function renderSamples(items){document.getElementById('samples').innerHTML=items.length?items.map(v=>`<div class="sample"><img src="/api/sample?profile=${profile.value}&id=${v.id}&t=${Date.now()}"><button class="danger" onclick="delSample('${v.id}')">Удалить</button></div>`).join(''):'<span class="small">Нет сохранённых кадров</span>'}function renderPoses(items){document.getElementById('poses').innerHTML=items.map((v,i)=>`<li>r${v.row} c${v.column} ${v.direction} <a href="#" onclick="delPose(${i});return false" style="color:#ff4d3d">×</a></li>`).join('')}
async function poll(){try{let q=await api('/api/config?profile='+profile.value);st.textContent=q.state;st.className=q.state;document.getElementById('active').textContent=q.active_profile==='bridge'?'МОСТ':'ОБЫЧНАЯ';document.getElementById('pose').textContent=q.pose?`r${q.pose.row} c${q.pose.column} ${q.pose.direction}`:'нет данных';document.getElementById('count').textContent=q.samples.length;if(!dirty&&drag<0)roi=q.profiles[profile.value].roi_points;renderSamples(q.samples);renderPoses(q.bridge_poses)}catch(e){}setTimeout(poll,1000)}
profile.onchange=()=>{dirty=false};document.getElementById('saveRoi').onclick=async()=>{try{await api('/api/profile',body({profile:profile.value,roi_points:roi}));dirty=false}catch(e){alert(e.message)}};document.getElementById('add').onclick=async()=>{try{await api('/api/samples',body({profile:profile.value}))}catch(e){alert(e.message)}};
async function delSample(id){if(confirm('Удалить этот кадр из калибровки?'))try{await api(`/api/samples?profile=${profile.value}&id=${id}`,{method:'DELETE'})}catch(e){alert(e.message)}}async function delPose(i){try{await api('/api/bridge_poses?index='+i,{method:'DELETE'})}catch(e){alert(e.message)}}document.getElementById('addPose').onclick=async()=>{try{await api('/api/bridge_poses',body({}))}catch(e){alert(e.message)}};frames();poll();
</script></body></html>'''.encode('utf-8')


def validate_roi_points(values) -> list[float]:
    points = [float(value) for value in values]
    if len(points) != 8 or any(value < 0.0 or value > 1.0 for value in points):
        raise ValueError('roi_points must contain eight values in [0, 1]')
    x0, y0, x1, y1, x2, y2, x3, y3 = points
    if not (x0 < x1 and x3 < x2 and (y0 + y1) < (y2 + y3)):
        raise ValueError('ROI points must be ordered top-left, top-right, bottom-right, bottom-left')
    area = .5 * abs(sum(points[i*2]*points[((i+1)%4)*2+1]-points[((i+1)%4)*2]*points[i*2+1] for i in range(4)))
    if area < .01: raise ValueError('Selected ROI is too small')
    return points


class AnnotatedStreamServer:
    def __init__(self, host='0.0.0.0', port=8080, roi_points=None,
                 on_roi_update: Callable[[list[float], str], None] | None = None,
                 on_action: Callable[[str, dict], dict | None] | None = None) -> None:
        self._condition=Condition();self._jpeg=None;self._source_jpeg=None;self._sequence=0;self._state='starting';self._clear_cell_count=0
        self._roi_points=validate_roi_points(roi_points or [.4,.68,.6,.68,.64,.9,.36,.9]);self._on_roi_update=on_roi_update;self._on_action=on_action;self._metadata={};owner=self
        class Handler(BaseHTTPRequestHandler):
            def send_bytes(self,data,kind,status=200):
                self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
            def send_json(self,value,status=200): self.send_bytes(json.dumps(value,ensure_ascii=False).encode(),'application/json',status)
            def payload(self):
                length=int(self.headers.get('Content-Length','0'))
                if length<0 or length>65536: raise ValueError('Invalid request size')
                return json.loads(self.rfile.read(length) or b'{}')
            def do_GET(self):
                parsed=urlsplit(self.path);path=parsed.path;query=parse_qs(parsed.query)
                if path in {'/','/index.html'}:self.send_bytes(INDEX_HTML,'text/html; charset=utf-8');return
                if path=='/api/config':
                    profile=query.get('profile',['normal'])[0]
                    with owner._condition:base={'state':owner._state,'roi_points':owner._roi_points,'sequence':owner._sequence,'clear_cell_count':owner._clear_cell_count,**owner._metadata}
                    if owner._on_action:base.update(owner._on_action('config',{'profile':profile}) or {})
                    self.send_json(base);return
                if path=='/api/sample' and owner._on_action:
                    result=owner._on_action('sample_path',{'profile':query.get('profile',[''])[0],'id':query.get('id',[''])[0]}) or {};target=Path(result.get('path',''))
                    if target.is_file():self.send_bytes(target.read_bytes(),'image/jpeg');return
                    self.send_error(404);return
                if path in {'/snapshot.jpg','/source.jpg'}:
                    with owner._condition:jpeg=owner._source_jpeg if path=='/source.jpg' else owner._jpeg
                    if jpeg is None:self.send_error(503,'No processed frame yet');return
                    self.send_bytes(jpeg,'image/jpeg');return
                if path=='/stream.mjpg':
                    self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.end_headers();last=-1
                    try:
                        while True:
                            with owner._condition:owner._condition.wait_for(lambda:owner._sequence!=last,2);jpeg,last=owner._jpeg,owner._sequence
                            if jpeg:self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'+f'Content-Length: {len(jpeg)}\r\n\r\n'.encode()+jpeg+b'\r\n')
                    except (BrokenPipeError,ConnectionResetError):pass
                    return
                self.send_error(404)
            def do_POST(self):
                try:
                    path=urlsplit(self.path).path;data=self.payload()
                    if path=='/api/roi':
                        points=validate_roi_points(data['roi_points']);mode=str(data.get('mode','append'))
                        if owner._on_roi_update:owner._on_roi_update(points,mode)
                        with owner._condition:owner._roi_points=points;owner._state='calibrating'
                        self.send_json({'accepted':True});return
                    actions={'/api/profile':'set_profile','/api/samples':'add_sample','/api/bridge_poses':'add_bridge_pose'}
                    if path in actions and owner._on_action:
                        if path=='/api/profile':data['roi_points']=validate_roi_points(data['roi_points'])
                        self.send_json(owner._on_action(actions[path],data) or {'accepted':True});return
                    self.send_error(404)
                except (KeyError,TypeError,ValueError,json.JSONDecodeError) as error:self.send_json({'error':str(error)},400)
            def do_DELETE(self):
                try:
                    parsed=urlsplit(self.path);q=parse_qs(parsed.query);actions={'/api/samples':'delete_sample','/api/bridge_poses':'delete_bridge_pose'}
                    if parsed.path in actions and owner._on_action:self.send_json(owner._on_action(actions[parsed.path],{k:v[0] for k,v in q.items()}) or {'accepted':True});return
                    self.send_error(404)
                except (KeyError,TypeError,ValueError) as error:self.send_json({'error':str(error)},400)
            def log_message(self,_format,*_args):pass
        self._server=ThreadingHTTPServer((host,int(port)),Handler);self._server.daemon_threads=True;self._thread=Thread(target=self._server.serve_forever,daemon=True)
    @property
    def port(self):return int(self._server.server_address[1])
    def start(self):self._thread.start()
    def update(self,jpeg,source_jpeg=None,state='unknown',clear_cell_count=0,metadata=None):
        with self._condition:
            self._jpeg=bytes(jpeg)
            if source_jpeg is not None:self._source_jpeg=bytes(source_jpeg)
            self._state=str(state);self._clear_cell_count=int(clear_cell_count);self._metadata=dict(metadata or {});self._sequence+=1;self._condition.notify_all()
    def set_status(self,state='unknown',metadata=None):
        with self._condition:
            self._state=str(state);self._metadata=dict(metadata or {});self._condition.notify_all()
    def stop(self):self._server.shutdown();self._server.server_close();self._thread.join(timeout=2)
