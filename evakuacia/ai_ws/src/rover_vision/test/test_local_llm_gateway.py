from pathlib import Path

import pytest

from rover_vision.local_llm_gateway import MODEL_SPECS, ModelManager, UnknownModelError


def manager(tmp_path: Path) -> ModelManager:
    return ModelManager(
        llama_server=Path("/bin/false"),
        models_dir=tmp_path,
        api_key="test-key",
        log_dir=tmp_path / "logs",
    )


def test_model_registry_has_three_unique_models(tmp_path):
    instance = manager(tmp_path)
    assert len(MODEL_SPECS) == 3
    assert len({spec.id for spec in MODEL_SPECS}) == 3
    assert instance.model_spec("qwen3.5-9b").parameters == "9B"


def test_unknown_model_is_rejected(tmp_path):
    with pytest.raises(UnknownModelError):
        manager(tmp_path).model_spec("not-a-model")


def test_status_reports_downloaded_files(tmp_path):
    instance = manager(tmp_path)
    spec = instance.model_spec("qwen3.5-2b")
    path = instance.model_path(spec)
    path.parent.mkdir(parents=True)
    with path.open("wb") as model_file:
        model_file.truncate(spec.expected_size)
    item = next(item for item in instance.status()["models"] if item["id"] == spec.id)
    assert item["downloaded"] is True
    assert item["file_size"] == spec.expected_size


def test_authenticated_proxy_forwards_payload_and_rejects_unknown_model(tmp_path):
    import json
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    from urllib.error import HTTPError
    from urllib.request import Request, urlopen

    from rover_vision.local_llm_gateway import make_handler

    received = []

    class Backend(BaseHTTPRequestHandler):
        def do_POST(self):
            payload = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            received.append((self.path, self.headers['Authorization'], payload))
            body = json.dumps({'choices': [{'message': {'content': 'ok'}}]}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_args):
            pass

    backend = ThreadingHTTPServer(('127.0.0.1', 0), Backend)
    instance = manager(tmp_path)
    instance.backend_port = backend.server_port
    selected = []
    # A fake model selection keeps this HTTP contract test independent of weights/GPU.
    instance.ensure_model = lambda model_id: selected.append(model_id)
    gateway = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(instance, 'test-key'))
    servers = (backend, gateway)
    threads = [threading.Thread(target=s.serve_forever, daemon=True) for s in servers]
    for thread in threads:
        thread.start()
    url = f'http://127.0.0.1:{gateway.server_port}/v1/chat/completions'
    payload = {'model': 'qwen3.5-9b', 'messages': [{'role': 'user', 'content': 'Привет'}]}

    def request(key, data=payload):
        return Request(url, data=json.dumps(data).encode(), headers={
            'Content-Type': 'application/json', 'Authorization': f'Bearer {key}',
        })

    try:
        for key in ('', 'wrong-key'):
            with pytest.raises(HTTPError) as error:
                urlopen(request(key), timeout=2)
            assert error.value.code == 401
        assert received == []
        assert selected == []
        with urlopen(request('test-key'), timeout=2) as response:
            assert json.load(response)['choices'][0]['message']['content'] == 'ok'
        assert selected == ['qwen3.5-9b']
        assert received == [('/v1/chat/completions', 'Bearer test-key', payload)]
        with pytest.raises(HTTPError) as error:
            urlopen(request('test-key', {'model': 'invalid'}), timeout=2)
        assert error.value.code == 400
        assert len(received) == 1
    finally:
        for server in servers:
            server.shutdown()
            server.server_close()
        for thread in threads:
            thread.join(timeout=2)
