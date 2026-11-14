from traffic_sign_detector.tcp_relay import FrameDecoder, pack_frame
from pathlib import Path


def test_frame_decoder_handles_fragmented_and_multiple_messages():
    decoder = FrameDecoder(max_payload_bytes=64)
    wire = pack_frame(b"first") + pack_frame(b"second")

    assert decoder.feed(wire[:3]) == []
    assert decoder.feed(wire[3:9]) == [b"first"]
    assert decoder.feed(wire[9:]) == [b"second"]


def test_frame_decoder_rejects_oversized_payload():
    decoder = FrameDecoder(max_payload_bytes=4)

    try:
        decoder.feed(pack_frame(b"12345"))
    except ValueError as exc:
        assert "too large" in str(exc)
    else:
        raise AssertionError("oversized relay frame was accepted")


def test_server_does_not_shadow_rclpy_node_clients_collection():
    source = (Path(__file__).parents[1] / "traffic_sign_detector" / "detection_tcp_server_node.py").read_text()
    assert "self._clients" not in source
