#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import serial
import whisper

from serial_whisper_bridge import (
    DEFAULT_PORT,
    HEADER,
    MAGIC as UPLINK_MAGIC,
    Segmenter,
    choose_device,
    normalize_model_name,
    resolve_serial_port,
    save_wav,
)


PCM_SAMPLE_RATE = 16000
FRAME_SAMPLES = 320
PLAYBACK_CHANNELS = 2
PLAYBACK_MAGIC = b"SPK1"
DEFAULT_REPLY_TEMPLATE = "Я услышал: {text}"
DEFAULT_TTS_VOICE = "Milena"
TARGET_TTS_PEAK = 28000.0


class ReplyEngine:
    def __init__(self, reply_template: str, agent_command: str | None, timeout_seconds: float) -> None:
        self._reply_template = reply_template
        self._agent_command = agent_command
        self._timeout_seconds = timeout_seconds

    def reply(self, text: str) -> str:
        if not self._agent_command:
            return self._reply_template.format(text=text)

        result = subprocess.run(
            self._agent_command,
            input=text,
            text=True,
            shell=True,
            capture_output=True,
            timeout=self._timeout_seconds,
            check=False,
        )
        if result.returncode != 0:
            stderr = result.stderr.strip()
            raise RuntimeError(stderr or f"agent command exited with code {result.returncode}")

        reply_text = result.stdout.strip()
        if not reply_text:
            return self._reply_template.format(text=text)
        return reply_text


class SerialVoiceBridge:
    def __init__(self, port: str, baud: int) -> None:
        self._serial = serial.Serial(port=port, baudrate=baud, timeout=0.25, write_timeout=1)
        self._buffer = bytearray()

    def close(self) -> None:
        self._serial.close()

    def read_uplink_frame(self) -> tuple[int, int, np.ndarray]:
        while True:
            chunk = self._serial.read(4096)
            if chunk:
                self._buffer.extend(chunk)

            while True:
                start = self._buffer.find(UPLINK_MAGIC)
                if start < 0:
                    if len(self._buffer) > len(UPLINK_MAGIC):
                        del self._buffer[:-len(UPLINK_MAGIC)]
                    break
                if start > 0:
                    del self._buffer[:start]
                if len(self._buffer) < HEADER.size:
                    break

                magic, sample_rate, sample_count, sequence = HEADER.unpack_from(self._buffer)
                if magic != UPLINK_MAGIC or sample_rate != PCM_SAMPLE_RATE or sample_count == 0 or sample_count > 4096:
                    del self._buffer[0]
                    continue

                packet_size = HEADER.size + sample_count * 2
                if len(self._buffer) < packet_size:
                    break

                payload = bytes(self._buffer[HEADER.size:packet_size])
                del self._buffer[:packet_size]
                frame = np.frombuffer(payload, dtype="<i2").copy()
                return sample_rate, sequence, frame

    def send_playback_samples(self, samples: np.ndarray) -> None:
        if samples.size == 0:
            return

        samples = samples.astype("<i2", copy=False)
        play_sequence = 0
        prebuffer_frames = 3
        next_deadline = time.monotonic()
        playback_chunk_samples = FRAME_SAMPLES * PLAYBACK_CHANNELS

        for offset in range(0, len(samples), playback_chunk_samples):
            chunk = samples[offset:offset + playback_chunk_samples]
            payload = chunk.tobytes()
            packet = HEADER.pack(PLAYBACK_MAGIC, PCM_SAMPLE_RATE, len(chunk), play_sequence) + payload
            self._serial.write(packet)
            self._serial.flush()
            play_sequence += 1

            if play_sequence <= prebuffer_frames:
                continue

            next_deadline += len(chunk) / (PCM_SAMPLE_RATE * PLAYBACK_CHANNELS)
            sleep_for = next_deadline - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)


def require_tool(name: str) -> str:
    tool = shutil.which(name)
    if tool is None:
        raise SystemExit(f"Required tool '{name}' was not found in PATH")
    return tool


def list_say_voices() -> int:
    say_bin = require_tool("say")
    result = subprocess.run([say_bin, "-v", "?"], text=True, capture_output=True, check=True)
    print(result.stdout.rstrip())
    return 0


def save_wav_with_channels(path: Path, sample_rate: int, samples: np.ndarray, channels: int) -> None:
    import wave

    with wave.open(str(path), "wb") as wav_file:
        wav_file.setnchannels(channels)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(samples.astype("<i2").tobytes())


def synthesize_reply_pcm(
    text: str,
    voice: str,
    rate: int,
    save_wav_path: Path | None,
) -> np.ndarray:
    say_bin = require_tool("say")
    ffmpeg_bin = require_tool("ffmpeg")

    with tempfile.TemporaryDirectory(prefix="voice-agent-") as tmpdir:
        tmp_path = Path(tmpdir)
        aiff_path = tmp_path / "reply.aiff"
        pcm_path = tmp_path / "reply.pcm"

        subprocess.run(
            [say_bin, "-v", voice, "-r", str(rate), "-o", str(aiff_path), text],
            text=True,
            capture_output=True,
            check=True,
        )
        subprocess.run(
            [
                ffmpeg_bin,
                "-v",
                "error",
                "-y",
                "-i",
                str(aiff_path),
                "-ac",
                "1",
                "-ar",
                str(PCM_SAMPLE_RATE),
                "-f",
                "s16le",
                str(pcm_path),
            ],
            text=True,
            capture_output=True,
            check=True,
        )

        samples = np.fromfile(pcm_path, dtype="<i2").copy()
        peak = float(np.max(np.abs(samples))) if samples.size else 0.0
        if peak > 0.0:
            gain = min(TARGET_TTS_PEAK / peak, 4.0)
            boosted = np.clip(samples.astype(np.float32) * gain, -32768.0, 32767.0)
            samples = boosted.astype("<i2")
        if samples.size:
            samples = np.column_stack((samples, samples)).reshape(-1).astype("<i2")
        if save_wav_path is not None:
            save_wav_path.parent.mkdir(parents=True, exist_ok=True)
            save_wav_with_channels(save_wav_path, PCM_SAMPLE_RATE, samples, PLAYBACK_CHANNELS)
        return samples


def log_event(
    role: str,
    text: str,
    append_file: Path | None,
    jsonl_file: Path | None,
) -> None:
    stamp = dt.datetime.now().isoformat(timespec="seconds")
    line = f"[{stamp}] {role}: {text}"
    print(line, flush=True)

    if append_file is not None:
        append_file.parent.mkdir(parents=True, exist_ok=True)
        with append_file.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")

    if jsonl_file is not None:
        jsonl_file.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "timestamp": stamp,
            "role": role,
            "text": text,
        }
        with jsonl_file.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload, ensure_ascii=False) + "\n")


def play_one_shot_text(args: argparse.Namespace) -> int:
    wav_path = None
    if args.save_reply_wavs_dir is not None:
        wav_path = args.save_reply_wavs_dir / "one_shot_reply.wav"
    samples = synthesize_reply_pcm(args.speak_text, args.tts_voice, args.tts_rate, wav_path)
    bridge = SerialVoiceBridge(resolve_serial_port(args.port), args.baud)
    try:
        print(f"Sending one-shot reply to {args.port}...", flush=True)
        bridge.send_playback_samples(samples)
    finally:
        bridge.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a simple voice assistant loop over the ESP32-S3 audio board.")
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
    parser.add_argument("--append-file", type=Path, default=None, help="Append both user and assistant messages to a plain text file.")
    parser.add_argument("--jsonl-file", type=Path, default=None, help="Append structured conversation events to a JSONL file.")
    parser.add_argument("--save-input-wavs-dir", type=Path, default=None, help="Save each detected user utterance as WAV.")
    parser.add_argument("--save-reply-wavs-dir", type=Path, default=None, help="Save each synthesized assistant reply as WAV.")
    parser.add_argument("--min-rms", type=float, default=350.0)
    parser.add_argument("--start-frames", type=int, default=3)
    parser.add_argument("--stop-frames", type=int, default=35)
    parser.add_argument("--pre-roll-frames", type=int, default=8)
    parser.add_argument("--max-utterance-seconds", type=float, default=12.0)
    parser.add_argument("--reply-template", default=DEFAULT_REPLY_TEMPLATE, help="Fallback reply template. Use {text} as the recognized text placeholder.")
    parser.add_argument("--agent-command", default=None, help="Optional shell command that receives the recognized text on stdin and returns the reply on stdout.")
    parser.add_argument("--agent-timeout", type=float, default=60.0, help="Timeout for --agent-command.")
    parser.add_argument("--tts-voice", default=DEFAULT_TTS_VOICE, help="macOS 'say' voice name used for spoken replies.")
    parser.add_argument("--tts-rate", type=int, default=175, help="Speech rate for macOS 'say'.")
    parser.add_argument("--list-voices", action="store_true", help="Print available macOS voices and exit.")
    parser.add_argument("--mute-reply", action="store_true", help="Do not synthesize or send spoken replies back to the board.")
    parser.add_argument("--speak-text", default=None, help="Send a one-shot spoken phrase to the board and exit.")
    args = parser.parse_args()

    if args.list_models:
        from serial_whisper_bridge import AVAILABLE_MODELS

        print("Supported Whisper models:")
        for model_name in AVAILABLE_MODELS:
            print(f"  - {model_name}")
        return 0

    if args.list_voices:
        return list_say_voices()

    if args.speak_text:
        return play_one_shot_text(args)

    args.port = resolve_serial_port(args.port)
    args.model = normalize_model_name(args.model)
    device = choose_device(args.device)
    print(f"Loading Whisper model '{args.model}' on {device}...", flush=True)
    model = whisper.load_model(args.model, device=device)

    bridge = SerialVoiceBridge(args.port, args.baud)
    reply_engine = ReplyEngine(args.reply_template, args.agent_command, args.agent_timeout)
    segmenter = Segmenter(
        frame_samples=FRAME_SAMPLES,
        min_rms=args.min_rms,
        start_frames=args.start_frames,
        stop_frames=args.stop_frames,
        pre_roll_frames=args.pre_roll_frames,
        max_utterance_seconds=args.max_utterance_seconds,
        sample_rate=PCM_SAMPLE_RATE,
    )

    utterance_index = 0
    print(f"Voice agent is listening on {args.port}...", flush=True)
    try:
        while True:
            sample_rate, sequence, frame = bridge.read_uplink_frame()
            frame_started_at = dt.datetime.now()
            utterance = segmenter.push(frame, frame_started_at)
            if utterance is None:
                continue

            if args.save_input_wavs_dir is not None:
                args.save_input_wavs_dir.mkdir(parents=True, exist_ok=True)
                wav_path = args.save_input_wavs_dir / f"{utterance_index:05d}_{sequence}_user.wav"
                save_wav(wav_path, sample_rate, utterance.samples)

            samples_f32 = utterance.samples.astype(np.float32) / 32768.0
            result = model.transcribe(
                samples_f32,
                language=args.language,
                task="transcribe",
                fp16=(device == "cuda"),
                temperature=0.0,
                condition_on_previous_text=False,
            )
            user_text = result.get("text", "").strip()
            if not user_text:
                continue

            log_event("user", user_text, args.append_file, args.jsonl_file)

            try:
                reply_text = reply_engine.reply(user_text)
            except Exception as exc:
                reply_text = f"Ошибка агента: {exc}"
            log_event("assistant", reply_text, args.append_file, args.jsonl_file)

            if args.mute_reply:
                utterance_index += 1
                continue

            reply_wav_path = None
            if args.save_reply_wavs_dir is not None:
                reply_wav_path = args.save_reply_wavs_dir / f"{utterance_index:05d}_{sequence}_assistant.wav"
            reply_samples = synthesize_reply_pcm(reply_text, args.tts_voice, args.tts_rate, reply_wav_path)
            bridge.send_playback_samples(reply_samples)
            utterance_index += 1
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
    finally:
        bridge.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
