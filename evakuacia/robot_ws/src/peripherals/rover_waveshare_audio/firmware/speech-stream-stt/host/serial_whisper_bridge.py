#!/usr/bin/env python3
from __future__ import annotations

import argparse
import collections
import dataclasses
import datetime as dt
import json
import socket
import struct
import sys
import time
import wave
from pathlib import Path

import numpy as np
import serial
import whisper
import torch


MAGIC = b"PCM1"
HEADER = struct.Struct("<4sHHI")
DEFAULT_PORT = "/dev/cu.usbmodem11401"
AVAILABLE_MODELS = (
    "tiny",
    "base",
    "small",
    "medium",
    "large",
    "turbo",
    "tiny.en",
    "base.en",
    "small.en",
    "medium.en",
    "large-v1",
    "large-v2",
    "large-v3",
)
MODEL_ALIASES = {
    "large-v3-turbo": "turbo",
}


@dataclasses.dataclass
class Utterance:
    started_at: dt.datetime
    ended_at: dt.datetime
    samples: np.ndarray


class TcpSink:
    def __init__(self, address: str | None) -> None:
        self._address = address
        self._sock: socket.socket | None = None

    def send(self, text: str) -> None:
        if not self._address:
            return
        if self._sock is None:
            host, port_text = self._address.rsplit(":", 1)
            self._sock = socket.create_connection((host, int(port_text)), timeout=3)
        try:
            self._sock.sendall((text + "\n").encode("utf-8"))
        except OSError:
            self.close()

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None


class SerialSink:
    def __init__(self, port: str | None, baud: int) -> None:
        self._port = port
        self._baud = baud
        self._serial: serial.Serial | None = None

    def send(self, text: str) -> None:
        if not self._port:
            return
        if self._serial is None:
            self._serial = serial.Serial(port=self._port, baudrate=self._baud, timeout=1, write_timeout=1)
        try:
            self._serial.write((text + "\n").encode("utf-8"))
            self._serial.flush()
        except serial.SerialException:
            self.close()

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None


class Segmenter:
    def __init__(
        self,
        frame_samples: int,
        min_rms: float,
        start_frames: int,
        stop_frames: int,
        pre_roll_frames: int,
        max_utterance_seconds: float,
        sample_rate: int,
    ) -> None:
        self.frame_samples = frame_samples
        self.min_rms = min_rms
        self.start_frames = start_frames
        self.stop_frames = stop_frames
        self.max_frames = int(max_utterance_seconds * sample_rate / frame_samples)
        self.pre_roll = collections.deque(maxlen=pre_roll_frames)
        self.in_speech = False
        self.pending_starts = 0
        self.pending_stops = 0
        self.noise_floor = min_rms
        self.frames: list[np.ndarray] = []
        self.started_at: dt.datetime | None = None

    def push(self, frame: np.ndarray, frame_started_at: dt.datetime) -> Utterance | None:
        rms = float(np.sqrt(np.mean(frame.astype(np.float32) ** 2)))
        threshold_on = max(self.min_rms, self.noise_floor * 3.0)
        threshold_off = max(self.min_rms * 0.75, self.noise_floor * 1.8)

        if not self.in_speech:
            self.pre_roll.append(frame.copy())
            if rms < threshold_on:
                self.noise_floor = self.noise_floor * 0.98 + max(rms, 1.0) * 0.02

            if rms > threshold_on:
                self.pending_starts += 1
            else:
                self.pending_starts = 0

            if self.pending_starts >= self.start_frames:
                self.in_speech = True
                self.pending_starts = 0
                self.pending_stops = 0
                self.frames = list(self.pre_roll)
                self.started_at = frame_started_at
            return None

        self.frames.append(frame.copy())
        if rms < threshold_off:
            self.pending_stops += 1
        else:
            self.pending_stops = 0

        if self.pending_stops >= self.stop_frames or len(self.frames) >= self.max_frames:
            utterance = np.concatenate(self.frames)
            result = Utterance(
                started_at=self.started_at or frame_started_at,
                ended_at=dt.datetime.now(),
                samples=utterance,
            )
            self.in_speech = False
            self.pending_stops = 0
            self.frames = []
            self.started_at = None
            return result

        return None


def choose_device(requested: str) -> str:
    if requested != "auto":
        return requested
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def normalize_model_name(model_name: str) -> str:
    normalized = MODEL_ALIASES.get(model_name, model_name)
    if normalized not in AVAILABLE_MODELS:
        available = ", ".join(AVAILABLE_MODELS)
        raise SystemExit(f"Unsupported Whisper model '{model_name}'. Available models: {available}")
    return normalized


def save_wav(path: Path, sample_rate: int, samples: np.ndarray) -> None:
    with wave.open(str(path), "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(samples.astype("<i2").tobytes())


def emit_text(
    text: str,
    started_at: dt.datetime,
    ended_at: dt.datetime,
    append_file: Path | None,
    jsonl_file: Path | None,
    tcp_sink: TcpSink,
    serial_sink: SerialSink,
) -> None:
    stamp = ended_at.isoformat(timespec="seconds")
    line = f"[{stamp}] {text}"
    print(line, flush=True)

    if append_file is not None:
        append_file.parent.mkdir(parents=True, exist_ok=True)
        with append_file.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")

    if jsonl_file is not None:
        jsonl_file.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "started_at": started_at.isoformat(),
            "ended_at": ended_at.isoformat(),
            "text": text,
        }
        with jsonl_file.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload, ensure_ascii=False) + "\n")

    tcp_sink.send(text)
    serial_sink.send(text)


def iter_pcm_frames(port: str, baud: int):
    ser = serial.Serial(port=port, baudrate=baud, timeout=1)
    buffer = bytearray()
    try:
        while True:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buffer.extend(chunk)

            while True:
                start = buffer.find(MAGIC)
                if start < 0:
                    if len(buffer) > len(MAGIC):
                        del buffer[:-len(MAGIC)]
                    break
                if start > 0:
                    del buffer[:start]
                if len(buffer) < HEADER.size:
                    break

                magic, sample_rate, sample_count, sequence = HEADER.unpack_from(buffer)
                if magic != MAGIC or sample_rate != 16000 or sample_count == 0 or sample_count > 4096:
                    del buffer[0]
                    continue

                packet_size = HEADER.size + sample_count * 2
                if len(buffer) < packet_size:
                    break

                payload = bytes(buffer[HEADER.size:packet_size])
                del buffer[:packet_size]
                frame = np.frombuffer(payload, dtype="<i2").copy()
                yield sample_rate, sequence, frame
    finally:
        ser.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Read framed PCM from ESP32-S3 over USB serial and transcribe it with Whisper.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument(
        "--model",
        default="turbo",
        help="Whisper model name. Examples: tiny, base, small, medium, large, turbo, tiny.en, base.en.",
    )
    parser.add_argument("--list-models", action="store_true", help="Print supported Whisper model names and exit.")
    parser.add_argument("--language", default=None, help="Force language, for example 'ru' or 'en'. Default is auto-detect.")
    parser.add_argument("--device", default="auto", help="Whisper device: auto, cpu, mps, cuda.")
    parser.add_argument("--append-file", type=Path, default=None, help="Append recognized text lines to a plain text file.")
    parser.add_argument("--jsonl-file", type=Path, default=None, help="Append structured transcript events to a JSONL file.")
    parser.add_argument("--save-wavs-dir", type=Path, default=None, help="Save each detected utterance as WAV for debugging.")
    parser.add_argument("--tcp", default=None, help="Forward recognized text to HOST:PORT over TCP.")
    parser.add_argument("--out-serial", default=None, help="Forward recognized text to another serial port as UTF-8 lines.")
    parser.add_argument("--out-serial-baud", type=int, default=115200, help="Baud rate for --out-serial.")
    parser.add_argument("--min-rms", type=float, default=350.0)
    parser.add_argument("--start-frames", type=int, default=3, help="How many loud frames in a row start an utterance.")
    parser.add_argument("--stop-frames", type=int, default=35, help="How many quiet frames in a row end an utterance.")
    parser.add_argument("--pre-roll-frames", type=int, default=8)
    parser.add_argument("--max-utterance-seconds", type=float, default=12.0)
    args = parser.parse_args()

    if args.list_models:
        print("Supported Whisper models:")
        for model_name in AVAILABLE_MODELS:
            print(f"  - {model_name}")
        return 0

    args.model = normalize_model_name(args.model)
    device = choose_device(args.device)
    print(f"Loading Whisper model '{args.model}' on {device}...", flush=True)
    model = whisper.load_model(args.model, device=device)

    tcp_sink = TcpSink(args.tcp)
    serial_sink = SerialSink(args.out_serial, args.out_serial_baud)
    segmenter = Segmenter(
        frame_samples=320,
        min_rms=args.min_rms,
        start_frames=args.start_frames,
        stop_frames=args.stop_frames,
        pre_roll_frames=args.pre_roll_frames,
        max_utterance_seconds=args.max_utterance_seconds,
        sample_rate=16000,
    )

    utterance_index = 0
    print(f"Listening on {args.port}...", flush=True)
    try:
        for sample_rate, sequence, frame in iter_pcm_frames(args.port, args.baud):
            frame_started_at = dt.datetime.now()
            utterance = segmenter.push(frame, frame_started_at)
            if utterance is None:
                continue

            samples_f32 = utterance.samples.astype(np.float32) / 32768.0
            result = model.transcribe(
                samples_f32,
                language=args.language,
                task="transcribe",
                fp16=(device == "cuda"),
                temperature=0.0,
                condition_on_previous_text=False,
            )
            text = result.get("text", "").strip()
            if not text:
                continue

            if args.save_wavs_dir is not None:
                args.save_wavs_dir.mkdir(parents=True, exist_ok=True)
                wav_path = args.save_wavs_dir / f"{utterance_index:05d}_{sequence}.wav"
                save_wav(wav_path, sample_rate, utterance.samples)

            emit_text(
                text,
                utterance.started_at,
                utterance.ended_at,
                args.append_file,
                args.jsonl_file,
                tcp_sink,
                serial_sink,
            )
            utterance_index += 1
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
    finally:
        tcp_sink.close()
        serial_sink.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
