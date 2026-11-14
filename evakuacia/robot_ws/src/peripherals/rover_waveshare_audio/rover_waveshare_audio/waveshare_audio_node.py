from __future__ import annotations

import dataclasses
import collections
import json
import os
import queue
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types
import wave
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rover_interfaces.srv import SpeakText
import serial
from serial import SerialException
from std_msgs.msg import String


MAGIC = b'PCM1'
PLAYBACK_MAGIC = b'SPK1'
HEADER = struct.Struct('<4sHHI')
PLAYBACK_CHANNELS = 2
DEFAULT_SAY_VOICE = 'Milena'
DEFAULT_ESPEAK_VOICE = 'ru'
DEFAULT_PIPER_MODEL = 'ru_RU-irina-medium'
DEFAULT_PIPER_DATA_DIR = '~/sverk_rover/tts_voices'
TARGET_TTS_PEAK = 28000.0
SUPPORTED_MODELS = {
    'tiny',
    'base',
    'small',
    'medium',
    'large',
    'turbo',
    'tiny.en',
    'base.en',
    'small.en',
    'medium.en',
    'large-v1',
    'large-v2',
    'large-v3',
    'large-v3-turbo',
}
MODEL_ALIASES = {
    'large-v3-turbo': 'turbo',
}


@dataclasses.dataclass
class Utterance:
    started_monotonic: float
    ended_monotonic: float
    sequence: int
    sample_rate: int
    samples: np.ndarray


class TcpSink:
    def __init__(self, address: str) -> None:
        self._address = address.strip()
        self._sock: socket.socket | None = None

    def send(self, text: str) -> None:
        if not self._address:
            return
        if self._sock is None:
            host, port_text = self._address.rsplit(':', 1)
            self._sock = socket.create_connection((host, int(port_text)), timeout=3.0)
        try:
            self._sock.sendall((text + '\n').encode('utf-8'))
        except OSError:
            self.close()

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None


class SerialSink:
    def __init__(self, port: str, baudrate: int) -> None:
        self._port = port.strip()
        self._baudrate = baudrate
        self._serial: serial.Serial | None = None

    def send(self, text: str) -> None:
        if not self._port:
            return
        if self._serial is None:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=1.0,
                write_timeout=1.0,
            )
        try:
            self._serial.write((text + '\n').encode('utf-8'))
            self._serial.flush()
        except (OSError, SerialException):
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
        sample_rate: int,
        min_rms: float,
        start_frames: int,
        stop_frames: int,
        pre_roll_frames: int,
        max_utterance_seconds: float,
    ) -> None:
        self.frame_samples = frame_samples
        self.sample_rate = sample_rate
        self.min_rms = min_rms
        self.start_frames = start_frames
        self.stop_frames = stop_frames
        self.max_frames = max(1, int(max_utterance_seconds * sample_rate / frame_samples))
        self.pre_roll: collections.deque[np.ndarray]
        self.pre_roll = collections.deque(maxlen=pre_roll_frames)
        self.in_speech = False
        self.pending_starts = 0
        self.pending_stops = 0
        self.noise_floor = min_rms
        self.frames: list[np.ndarray] = []
        self.started_monotonic = 0.0

    def push(self, frame: np.ndarray, sequence: int) -> Utterance | None:
        rms = float(np.sqrt(np.mean(frame.astype(np.float32) ** 2)))
        threshold_on = max(self.min_rms, self.noise_floor * 3.0)
        threshold_off = max(self.min_rms * 0.75, self.noise_floor * 1.8)
        now = time.monotonic()

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
                self.started_monotonic = now
            return None

        self.frames.append(frame.copy())
        if rms < threshold_off:
            self.pending_stops += 1
        else:
            self.pending_stops = 0

        if self.pending_stops >= self.stop_frames or len(self.frames) >= self.max_frames:
            samples = np.concatenate(self.frames)
            utterance = Utterance(
                started_monotonic=self.started_monotonic,
                ended_monotonic=now,
                sequence=sequence,
                sample_rate=self.sample_rate,
                samples=samples,
            )
            self.in_speech = False
            self.pending_stops = 0
            self.frames = []
            self.started_monotonic = 0.0
            return utterance

        return None


def normalize_model_name(model_name: str) -> str:
    normalized = MODEL_ALIASES.get(model_name.strip(), model_name.strip())
    if normalized not in SUPPORTED_MODELS:
        available = ', '.join(sorted(SUPPORTED_MODELS))
        raise ValueError(f'Unsupported Whisper model {model_name!r}. Available: {available}')
    return normalized


def choose_device(requested: str) -> str:
    requested = requested.strip().lower()
    if requested != 'auto':
        return requested

    try:
        import torch
    except ImportError:
        return 'cpu'

    if torch.backends.mps.is_available():
        return 'mps'
    if torch.cuda.is_available():
        return 'cuda'
    return 'cpu'


def save_wav(path: Path, sample_rate: int, samples: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(samples.astype('<i2').tobytes())


def save_wav_with_channels(
    path: Path,
    sample_rate: int,
    samples: np.ndarray,
    channels: int,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), 'wb') as wav_file:
        wav_file.setnchannels(channels)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(samples.astype('<i2').tobytes())


def require_tool(name: str) -> str:
    tool = shutil.which(name)
    if tool is None:
        raise RuntimeError(f"Required tool '{name}' was not found in PATH")
    return tool


def prepare_whisper_import() -> None:
    os.environ.setdefault('NUMBA_JIT_COVERAGE', '0')

    try:
        import coverage
    except Exception:  # noqa: BLE001
        return

    coverage_types = getattr(coverage, 'types', None)
    if coverage_types is None:
        coverage_types = types.SimpleNamespace()
        setattr(coverage, 'types', coverage_types)

    if not hasattr(coverage_types, 'Tracer'):
        tracer_type = getattr(coverage_types, 'TTracer', object)
        setattr(coverage_types, 'Tracer', tracer_type)

    # Numba 0.62 imports coverage type hints at runtime. Older coverage.py 7.x
    # releases do not expose every alias that Numba expects, even when JIT
    # coverage is disabled. These aliases are only needed to complete import.
    for alias in (
        'TTraceData',
        'TShouldTraceFn',
        'TFileDisposition',
        'TShouldStartContextFn',
        'TWarnFn',
        'TTraceFn',
    ):
        if not hasattr(coverage_types, alias):
            setattr(coverage_types, alias, Any)


class WaveshareAudioNode(Node):
    def __init__(self) -> None:
        super().__init__('waveshare_audio_node')

        self.declare_parameter('serial_device', '/dev/waveshare_audio')
        self.declare_parameter('baudrate', 2_000_000)
        self.declare_parameter('read_timeout_sec', 1.0)
        self.declare_parameter('reconnect_interval_sec', 2.0)
        self.declare_parameter('expected_sample_rate', 16000)
        self.declare_parameter('expected_frame_samples', 320)
        self.declare_parameter('output_topic', '/voice/text')
        self.declare_parameter('status_topic', '/waveshare_audio/status')
        self.declare_parameter('transcript_json_topic', '/voice/transcript')
        self.declare_parameter('publish_transcript_json', True)
        self.declare_parameter('whisper_model', 'base')
        self.declare_parameter('language', 'ru')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('min_rms', 350.0)
        self.declare_parameter('start_frames', 3)
        self.declare_parameter('stop_frames', 35)
        self.declare_parameter('pre_roll_frames', 8)
        self.declare_parameter('max_utterance_seconds', 12.0)
        self.declare_parameter('utterance_queue_size', 4)
        self.declare_parameter('condition_on_previous_text', False)
        self.declare_parameter('temperature', 0.0)
        self.declare_parameter('save_wavs_dir', '')
        self.declare_parameter('append_file', '')
        self.declare_parameter('jsonl_file', '')
        self.declare_parameter('tcp_sink', '')
        self.declare_parameter('serial_sink_device', '')
        self.declare_parameter('serial_sink_baudrate', 115200)
        self.declare_parameter('enable_tts', True)
        self.declare_parameter('tts_service_name', '/voice/say')
        self.declare_parameter('tts_engine', 'piper')
        self.declare_parameter('tts_voice', '')
        self.declare_parameter('tts_rate', 175)
        self.declare_parameter('tts_target_peak', TARGET_TTS_PEAK)
        self.declare_parameter('tts_gain_limit', 4.0)
        self.declare_parameter('tts_queue_size', 4)
        self.declare_parameter('save_tts_wavs_dir', '')
        self.declare_parameter('piper_module', 'piper')
        self.declare_parameter('piper_model', DEFAULT_PIPER_MODEL)
        self.declare_parameter('piper_data_dir', DEFAULT_PIPER_DATA_DIR)

        self.serial_device = str(self.get_parameter('serial_device').value).strip()
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.read_timeout = max(0.05, float(self.get_parameter('read_timeout_sec').value))
        self.reconnect_interval = max(
            0.2,
            float(self.get_parameter('reconnect_interval_sec').value),
        )
        self.expected_sample_rate = int(self.get_parameter('expected_sample_rate').value)
        self.expected_frame_samples = int(
            self.get_parameter('expected_frame_samples').value
        )
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.transcript_json_topic = str(
            self.get_parameter('transcript_json_topic').value
        )
        self.publish_transcript_json = bool(
            self.get_parameter('publish_transcript_json').value
        )
        self.whisper_model_name = normalize_model_name(
            str(self.get_parameter('whisper_model').value)
        )
        language = str(self.get_parameter('language').value).strip()
        self.language = language if language else None
        self.whisper_device = choose_device(str(self.get_parameter('device').value))
        self.condition_on_previous_text = bool(
            self.get_parameter('condition_on_previous_text').value
        )
        self.temperature = float(self.get_parameter('temperature').value)
        self.save_wavs_dir = Path(str(self.get_parameter('save_wavs_dir').value)).expanduser()
        self.save_wavs_enabled = bool(str(self.get_parameter('save_wavs_dir').value).strip())
        self.append_file = Path(str(self.get_parameter('append_file').value)).expanduser()
        self.append_file_enabled = bool(str(self.get_parameter('append_file').value).strip())
        self.jsonl_file = Path(str(self.get_parameter('jsonl_file').value)).expanduser()
        self.jsonl_file_enabled = bool(str(self.get_parameter('jsonl_file').value).strip())
        self.tcp_sink = TcpSink(str(self.get_parameter('tcp_sink').value))
        self.serial_sink = SerialSink(
            str(self.get_parameter('serial_sink_device').value),
            int(self.get_parameter('serial_sink_baudrate').value),
        )
        self.tts_enabled = bool(self.get_parameter('enable_tts').value)
        self.tts_service_name = str(
            self.get_parameter('tts_service_name').value
        ).strip()
        self.tts_engine = str(self.get_parameter('tts_engine').value).strip().lower()
        self.tts_voice = str(self.get_parameter('tts_voice').value).strip()
        self.tts_rate = int(self.get_parameter('tts_rate').value)
        self.tts_target_peak = max(
            1.0,
            float(self.get_parameter('tts_target_peak').value),
        )
        self.tts_gain_limit = max(
            1.0,
            float(self.get_parameter('tts_gain_limit').value),
        )
        self.save_tts_wavs_dir = Path(
            str(self.get_parameter('save_tts_wavs_dir').value)
        ).expanduser()
        self.save_tts_wavs_enabled = bool(
            str(self.get_parameter('save_tts_wavs_dir').value).strip()
        )
        self.piper_module = str(
            self.get_parameter('piper_module').value
        ).strip() or 'piper'
        self.piper_model = str(
            self.get_parameter('piper_model').value
        ).strip() or DEFAULT_PIPER_MODEL
        self.piper_data_dir = Path(
            str(self.get_parameter('piper_data_dir').value)
        ).expanduser()

        if not self.serial_device:
            raise ValueError('serial_device must not be empty')
        if self.baudrate <= 0:
            raise ValueError('baudrate must be positive')
        if self.tts_enabled and not self.tts_service_name:
            raise ValueError('tts_service_name must not be empty when enable_tts is true')
        if self.tts_enabled and self.tts_rate <= 0:
            raise ValueError('tts_rate must be positive')

        self.text_pub = self.create_publisher(String, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.transcript_json_pub = self.create_publisher(
            String,
            self.transcript_json_topic,
            10,
        )

        self.segmenter = Segmenter(
            frame_samples=self.expected_frame_samples,
            sample_rate=self.expected_sample_rate,
            min_rms=float(self.get_parameter('min_rms').value),
            start_frames=int(self.get_parameter('start_frames').value),
            stop_frames=int(self.get_parameter('stop_frames').value),
            pre_roll_frames=int(self.get_parameter('pre_roll_frames').value),
            max_utterance_seconds=float(
                self.get_parameter('max_utterance_seconds').value
            ),
        )
        self.utterance_queue: queue.Queue[Utterance] = queue.Queue(
            maxsize=max(1, int(self.get_parameter('utterance_queue_size').value)),
        )
        self.tts_queue: queue.Queue[str] = queue.Queue(
            maxsize=max(1, int(self.get_parameter('tts_queue_size').value)),
        )

        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._playback_active = threading.Event()
        self._connected = False
        self._last_status = ''
        self._last_connect_warning = 0.0
        self._utterance_index = 0
        self._tts_index = 0

        self._reader_thread = threading.Thread(
            target=self._read_loop,
            name='waveshare-audio-pcm-reader',
            daemon=True,
        )
        self._transcriber_thread = threading.Thread(
            target=self._transcribe_loop,
            name='waveshare-audio-whisper',
            daemon=True,
        )
        self._reader_thread.start()
        self._transcriber_thread.start()
        self._tts_service = None
        self._tts_thread: threading.Thread | None = None
        if self.tts_enabled:
            self._tts_service = self.create_service(
                SpeakText,
                self.tts_service_name,
                self._handle_speak_text,
            )
            self._tts_thread = threading.Thread(
                target=self._tts_loop,
                name='waveshare-audio-tts-playback',
                daemon=True,
            )
            self._tts_thread.start()
        self.create_timer(1.0, self._publish_periodic_status)

        self.get_logger().info(
            'Waveshare Whisper STT node started: '
            f'{self.serial_device} @ {self.baudrate}, '
            f'model={self.whisper_model_name}, device={self.whisper_device}, '
            f'output_topic={self.output_topic}, '
            f'tts_service={self.tts_service_name if self.tts_enabled else "disabled"}'
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._close_serial()
        self.tcp_sink.close()
        self.serial_sink.close()
        threads = [self._reader_thread, self._transcriber_thread]
        if self._tts_thread is not None:
            threads.append(self._tts_thread)
        for thread in threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
        return super().destroy_node()

    def _open_serial(self) -> serial.Serial | None:
        try:
            serial_port = serial.Serial(
                self.serial_device,
                self.baudrate,
                timeout=self.read_timeout,
                write_timeout=1.0,
            )
            serial_port.reset_input_buffer()
            self._set_connected(True, f'connected {self.serial_device}')
            return serial_port
        except (OSError, SerialException) as exc:
            now = time.monotonic()
            if now - self._last_connect_warning > 5.0:
                self.get_logger().warn(
                    f'Unable to open Waveshare audio serial device '
                    f'{self.serial_device}: {exc}'
                )
                self._last_connect_warning = now
            self._set_connected(False, f'disconnected {self.serial_device}: {exc}')
            return None

    def _close_serial(self) -> None:
        with self._serial_lock:
            serial_port = self._serial
            self._serial = None

        if serial_port is not None:
            try:
                serial_port.close()
            except (OSError, SerialException):
                pass
        self._set_connected(False, f'disconnected {self.serial_device}')

    def _read_loop(self) -> None:
        buffer = bytearray()
        while not self._stop_event.is_set():
            if self._serial is None:
                with self._serial_lock:
                    if self._serial is None:
                        self._serial = self._open_serial()
                if self._serial is None:
                    self._stop_event.wait(self.reconnect_interval)
                    continue
                buffer.clear()

            try:
                if self._playback_active.is_set():
                    self._stop_event.wait(0.02)
                    continue

                with self._serial_lock:
                    serial_port = self._serial
                    if serial_port is None:
                        continue
                    chunk = serial_port.read(4096)
                if not chunk:
                    continue
                buffer.extend(chunk)
                self._consume_buffer(buffer)
            except (OSError, SerialException) as exc:
                self.get_logger().warn(f'Waveshare audio serial read failed: {exc}')
                self._close_serial()
                self._stop_event.wait(self.reconnect_interval)

    def _consume_buffer(self, buffer: bytearray) -> None:
        while not self._stop_event.is_set():
            start = buffer.find(MAGIC)
            if start < 0:
                if len(buffer) > len(MAGIC):
                    del buffer[:-len(MAGIC)]
                return
            if start > 0:
                del buffer[:start]
            if len(buffer) < HEADER.size:
                return

            magic, sample_rate, sample_count, sequence = HEADER.unpack_from(buffer)
            if (
                magic != MAGIC
                or sample_rate != self.expected_sample_rate
                or sample_count <= 0
                or sample_count != self.expected_frame_samples
                or sample_count > 4096
            ):
                del buffer[0]
                continue

            packet_size = HEADER.size + sample_count * 2
            if len(buffer) < packet_size:
                return

            payload = bytes(buffer[HEADER.size:packet_size])
            del buffer[:packet_size]

            frame = np.frombuffer(payload, dtype='<i2').copy()
            utterance = self.segmenter.push(frame, sequence)
            if utterance is not None:
                self._enqueue_utterance(utterance)

    def _enqueue_utterance(self, utterance: Utterance) -> None:
        try:
            self.utterance_queue.put_nowait(utterance)
            self._publish_status(
                f'queued utterance seq={utterance.sequence} '
                f'duration={len(utterance.samples) / utterance.sample_rate:.2f}s'
            )
        except queue.Full:
            self.get_logger().warn('Dropping utterance: Whisper queue is full')
            self._publish_status('dropping utterance: whisper queue full')

    def _transcribe_loop(self) -> None:
        model = None
        while not self._stop_event.is_set() and model is None:
            try:
                prepare_whisper_import()
                import whisper

                self._publish_status(
                    f'loading whisper model {self.whisper_model_name} '
                    f'on {self.whisper_device}'
                )
                model = whisper.load_model(
                    self.whisper_model_name,
                    device=self.whisper_device,
                )
                self._publish_status(
                    f'whisper ready model={self.whisper_model_name} '
                    f'device={self.whisper_device}'
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    'Unable to load Whisper. Install dependencies with '
                    '`python3 -m pip install -U openai-whisper torch`: '
                    f'{exc}'
                )
                self._publish_status(f'whisper load failed: {exc}')
                self._stop_event.wait(5.0)

        while not self._stop_event.is_set():
            try:
                utterance = self.utterance_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if model is None:
                continue

            try:
                self._transcribe_utterance(model, utterance)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Whisper transcription failed: {exc}')
                self._publish_status(f'transcription failed: {exc}')

    def _transcribe_utterance(self, model: Any, utterance: Utterance) -> None:
        samples_f32 = utterance.samples.astype(np.float32) / 32768.0
        result = model.transcribe(
            samples_f32,
            language=self.language,
            task='transcribe',
            fp16=(self.whisper_device == 'cuda'),
            temperature=self.temperature,
            condition_on_previous_text=self.condition_on_previous_text,
        )
        text = str(result.get('text', '')).strip()
        if not text:
            self._publish_status('empty transcript')
            return

        duration = len(utterance.samples) / utterance.sample_rate
        self._publish_text(text)
        self._publish_transcript_json(text, utterance, duration, result)
        self._write_optional_outputs(text, utterance, duration)
        self.tcp_sink.send(text)
        self.serial_sink.send(text)
        self._publish_status(f'transcribed: {text}')

    def _handle_speak_text(
        self,
        request: SpeakText.Request,
        response: SpeakText.Response,
    ) -> SpeakText.Response:
        text = str(request.text or '').strip()
        if not text:
            response.accepted = False
            response.message = 'text is empty'
            return response

        try:
            self.tts_queue.put_nowait(text)
            response.accepted = True
            response.message = 'queued'
            self._publish_status(f'queued tts text: {text}')
        except queue.Full:
            self.get_logger().warn('Dropping TTS text: playback queue is full')
            response.accepted = False
            response.message = 'playback queue is full'
            self._publish_status('dropping tts text: playback queue full')
        return response

    def _tts_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                text = self.tts_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                samples = self._synthesize_tts(text)
                self._send_playback_samples(samples)
                self._publish_status(f'tts spoken: {text}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'TTS playback failed: {exc}')
                self._publish_status(f'tts failed: {exc}')
            finally:
                self._tts_index += 1

    def _synthesize_tts(self, text: str) -> np.ndarray:
        engine = self._resolve_tts_engine()
        ffmpeg_bin = require_tool('ffmpeg')

        with tempfile.TemporaryDirectory(prefix='rover-waveshare-tts-') as tmp_dir:
            tmp_path = Path(tmp_dir)
            pcm_path = tmp_path / 'speech.pcm'

            if engine == 'say':
                input_audio_path = self._synthesize_with_say(text, tmp_path)
            elif engine == 'espeak-ng':
                input_audio_path = self._synthesize_with_espeak_ng(text, tmp_path)
            elif engine == 'piper':
                input_audio_path = self._synthesize_with_piper(text, tmp_path)
            else:
                raise RuntimeError(
                    f'Unsupported tts_engine {self.tts_engine!r}. '
                    'Use auto, piper, say, or espeak-ng.'
                )

            self._run_subprocess([
                ffmpeg_bin,
                '-v',
                'error',
                '-y',
                '-i',
                str(input_audio_path),
                '-ac',
                '1',
                '-ar',
                str(self.expected_sample_rate),
                '-f',
                's16le',
                str(pcm_path),
            ])

            mono_samples = np.fromfile(pcm_path, dtype='<i2').copy()
            if mono_samples.size == 0:
                return mono_samples

            mono_samples = self._normalize_tts_samples(mono_samples)
            stereo_samples = (
                np.column_stack((mono_samples, mono_samples)).reshape(-1).astype('<i2')
            )

            if self.save_tts_wavs_enabled:
                wav_path = self.save_tts_wavs_dir / f'{self._tts_index:05d}.wav'
                save_wav_with_channels(
                    wav_path,
                    self.expected_sample_rate,
                    stereo_samples,
                    PLAYBACK_CHANNELS,
                )

            return stereo_samples

    def _resolve_tts_engine(self) -> str:
        requested = self.tts_engine or 'auto'
        if requested != 'auto':
            return requested

        if (
            self.piper_data_dir.is_dir()
            and shutil.which('ffmpeg') is not None
        ):
            return 'piper'
        if shutil.which('say') is not None and shutil.which('ffmpeg') is not None:
            return 'say'
        if shutil.which('espeak-ng') is not None and shutil.which('ffmpeg') is not None:
            return 'espeak-ng'
        raise RuntimeError(
            'No supported TTS backend found. Install piper-tts with a voice '
            'model, or install ffmpeg and espeak-ng on Raspberry Pi.'
        )

    def _synthesize_with_piper(self, text: str, tmp_path: Path) -> Path:
        if not self.piper_data_dir.is_dir():
            raise RuntimeError(
                f'Piper data dir not found: {self.piper_data_dir}. '
                'Run the rover_waveshare_audio tools/install_piper_ru_voice.sh script.'
            )
        model_path = self.piper_data_dir / f'{self.piper_model}.onnx'
        if not model_path.is_file():
            raise RuntimeError(
                f'Piper model not found: {model_path}. '
                'Run the rover_waveshare_audio tools/install_piper_ru_voice.sh script.'
            )

        audio_path = tmp_path / 'speech.wav'
        command = [
            sys.executable,
            '-m',
            self.piper_module,
            '-m',
            self.piper_model,
            '--data-dir',
            str(self.piper_data_dir),
            '-f',
            str(audio_path),
            '--',
            text,
        ]

        self._run_subprocess(command)
        return audio_path

    def _synthesize_with_say(self, text: str, tmp_path: Path) -> Path:
        say_bin = require_tool('say')
        audio_path = tmp_path / 'speech.aiff'
        voice = self.tts_voice or DEFAULT_SAY_VOICE
        self._run_subprocess([
            say_bin,
            '-v',
            voice,
            '-r',
            str(self.tts_rate),
            '-o',
            str(audio_path),
            text,
        ])
        return audio_path

    def _synthesize_with_espeak_ng(self, text: str, tmp_path: Path) -> Path:
        espeak_bin = require_tool('espeak-ng')
        audio_path = tmp_path / 'speech.wav'
        voice = self.tts_voice or self.language or DEFAULT_ESPEAK_VOICE
        self._run_subprocess([
            espeak_bin,
            '-v',
            voice,
            '-s',
            str(self.tts_rate),
            '-w',
            str(audio_path),
            text,
        ])
        return audio_path

    def _normalize_tts_samples(self, samples: np.ndarray) -> np.ndarray:
        peak = float(np.max(np.abs(samples))) if samples.size else 0.0
        if peak <= 0.0:
            return samples.astype('<i2', copy=False)

        gain = min(self.tts_target_peak / peak, self.tts_gain_limit)
        boosted = np.clip(samples.astype(np.float32) * gain, -32768.0, 32767.0)
        return boosted.astype('<i2')

    def _send_playback_samples(self, samples: np.ndarray) -> None:
        if samples.size == 0:
            self._publish_status('tts produced empty audio')
            return

        samples = samples.astype('<i2', copy=False)
        playback_chunk_samples = self.expected_frame_samples * PLAYBACK_CHANNELS
        play_sequence = 0
        prebuffer_frames = 3
        next_deadline = time.monotonic()

        self._playback_active.set()
        self._cancel_serial_read()
        try:
            for offset in range(0, len(samples), playback_chunk_samples):
                if self._stop_event.is_set():
                    return

                chunk = samples[offset:offset + playback_chunk_samples]
                packet = (
                    HEADER.pack(
                        PLAYBACK_MAGIC,
                        self.expected_sample_rate,
                        len(chunk),
                        play_sequence,
                    )
                    + chunk.tobytes()
                )
                self._write_playback_packet(packet)
                play_sequence += 1

                if play_sequence <= prebuffer_frames:
                    continue

                next_deadline += len(chunk) / (
                    self.expected_sample_rate * PLAYBACK_CHANNELS
                )
                sleep_for = next_deadline - time.monotonic()
                if sleep_for > 0:
                    self._stop_event.wait(sleep_for)
        finally:
            self._playback_active.clear()

    def _write_playback_packet(self, packet: bytes) -> None:
        try:
            with self._serial_lock:
                if self._serial is None:
                    self._serial = self._open_serial()
                serial_port = self._serial
                if serial_port is None:
                    raise RuntimeError(f'Unable to open {self.serial_device}')
                serial_port.write(packet)
                serial_port.flush()
        except (OSError, SerialException) as exc:
            self._close_serial()
            raise RuntimeError(f'Waveshare audio serial write failed: {exc}') from exc

    def _cancel_serial_read(self) -> None:
        serial_port = self._serial
        if serial_port is None:
            return

        cancel_read = getattr(serial_port, 'cancel_read', None)
        if not callable(cancel_read):
            return

        try:
            cancel_read()
        except (OSError, SerialException):
            pass

    @staticmethod
    def _run_subprocess(command: list[str], input_text: str | None = None) -> None:
        result = subprocess.run(
            command,
            input=input_text,
            text=True,
            capture_output=True,
            check=False,
        )
        if result.returncode != 0:
            stderr = result.stderr.strip()
            stdout = result.stdout.strip()
            detail = stderr or stdout or f'exit code {result.returncode}'
            raise RuntimeError(detail)

    def _write_optional_outputs(
        self,
        text: str,
        utterance: Utterance,
        duration: float,
    ) -> None:
        payload = {
            'text': text,
            'sequence': utterance.sequence,
            'sample_rate': utterance.sample_rate,
            'duration_sec': duration,
            'model': self.whisper_model_name,
            'language': self.language,
        }

        if self.save_wavs_enabled:
            wav_path = (
                self.save_wavs_dir
                / f'{self._utterance_index:05d}_{utterance.sequence}.wav'
            )
            save_wav(wav_path, utterance.sample_rate, utterance.samples)

        if self.append_file_enabled:
            self.append_file.parent.mkdir(parents=True, exist_ok=True)
            with self.append_file.open('a', encoding='utf-8') as stream:
                stream.write(text + '\n')

        if self.jsonl_file_enabled:
            self.jsonl_file.parent.mkdir(parents=True, exist_ok=True)
            with self.jsonl_file.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(payload, ensure_ascii=False) + '\n')

        self._utterance_index += 1

    def _publish_transcript_json(
        self,
        text: str,
        utterance: Utterance,
        duration: float,
        result: dict[str, Any],
    ) -> None:
        if not self.publish_transcript_json:
            return

        payload = {
            'text': text,
            'sequence': utterance.sequence,
            'sample_rate': utterance.sample_rate,
            'duration_sec': duration,
            'model': self.whisper_model_name,
            'language': self.language,
            'segments': result.get('segments', []),
        }
        self._publish_string(self.transcript_json_pub, json.dumps(payload, ensure_ascii=False))

    def _set_connected(self, connected: bool, status: str) -> None:
        if connected != self._connected:
            self._connected = connected
            if connected:
                self.get_logger().info(status)
            else:
                self.get_logger().warn(status)
        self._last_status = status

    def _publish_periodic_status(self) -> None:
        if self._last_status:
            self._publish_status(self._last_status)

    def _publish_status(self, value: str) -> None:
        self._last_status = value
        self._publish_string(self.status_pub, value)

    def _publish_text(self, value: str) -> None:
        self._publish_string(self.text_pub, value)

    @staticmethod
    def _publish_string(publisher, value: str) -> None:
        msg = String()
        msg.data = value
        publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaveshareAudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
