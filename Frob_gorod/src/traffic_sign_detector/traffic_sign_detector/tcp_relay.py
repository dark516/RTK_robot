import struct


def pack_frame(payload: bytes) -> bytes:
    return struct.pack("!I", len(payload)) + payload


class FrameDecoder:
    def __init__(self, max_payload_bytes: int = 1_048_576):
        self._buffer = bytearray()
        self._max_payload_bytes = int(max_payload_bytes)

    def feed(self, data: bytes) -> list[bytes]:
        self._buffer.extend(data)
        messages = []
        while len(self._buffer) >= 4:
            size = struct.unpack("!I", self._buffer[:4])[0]
            if size > self._max_payload_bytes:
                raise ValueError(f"relay payload too large: {size} bytes")
            if len(self._buffer) < 4 + size:
                break
            messages.append(bytes(self._buffer[4 : 4 + size]))
            del self._buffer[: 4 + size]
        return messages
