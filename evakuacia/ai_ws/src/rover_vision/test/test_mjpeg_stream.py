import json
from urllib.request import Request, urlopen

from rover_vision.mjpeg_stream import AnnotatedStreamServer


def test_index_and_snapshot_are_served():
    server = AnnotatedStreamServer('127.0.0.1', 0)
    server.start()
    try:
        server.update(b'jpeg-test-data')
        with urlopen(f'http://127.0.0.1:{server.port}/', timeout=2) as response:
            assert response.status == 200
            assert b'stream.mjpg' in response.read()
        with urlopen(
            f'http://127.0.0.1:{server.port}/snapshot.jpg', timeout=2
        ) as response:
            assert response.headers['Content-Type'] == 'image/jpeg'
            assert response.read() == b'jpeg-test-data'
    finally:
        server.stop()


def test_roi_api_validates_and_forwards_points():
    received = []
    initial = [0.40, 0.68, 0.60, 0.68, 0.64, 0.90, 0.36, 0.90]
    updated = [0.41, 0.67, 0.59, 0.67, 0.63, 0.89, 0.37, 0.89]
    server = AnnotatedStreamServer(
        '127.0.0.1', 0, initial, lambda points, mode: received.append((points, mode))
    )
    server.start()
    try:
        request = Request(
            f'http://127.0.0.1:{server.port}/api/roi',
            data=json.dumps({'roi_points': updated, 'mode': 'append'}).encode(),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with urlopen(request, timeout=2) as response:
            assert response.status == 200
        with urlopen(
            f'http://127.0.0.1:{server.port}/api/config', timeout=2
        ) as response:
            config = json.load(response)
        assert received == [(updated, 'append')]
        assert config['roi_points'] == updated
        assert config['state'] == 'calibrating'
    finally:
        server.stop()
