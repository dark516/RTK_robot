# rover_waveshare_audio

ROS 2 Whisper STT/TTS bridge for the Waveshare `ESP32-S3-AUDIO-Board`.

The ESP32 firmware streams framed `16 kHz mono s16le` PCM packets over USB
Serial/JTAG using the same `PCM1` protocol as the working `voice test`
prototype. The ROS node reads those frames, detects utterances, transcribes
them with Whisper, and publishes recognized text.

The same serial link can also play speech back through the board speaker. The
ROS node exposes a configurable service, synthesizes the requested text on the
Raspberry Pi, converts it to `16 kHz stereo s16le`, and sends `SPK1` playback
frames back to the ESP32.

## Topics

- `/voice/text` (`std_msgs/msg/String`) recognized text.
- `/voice/transcript` (`std_msgs/msg/String`) JSON transcript metadata.
- `/waveshare_audio/status` (`std_msgs/msg/String`) connection/STT status.

## Services

- `/voice/say` (`rover_interfaces/srv/SpeakText`) text to speak through the module.

Standalone defaults live in `config/default.example.yaml`. During normal rover
bringup, topic and service names come from `rover_bringup/config`.

## Firmware

The source prototypes from `voice test` are stored in `firmware/`:

- `firmware/speech-stream-stt`: recommended firmware for ROS STT/TTS.
- `firmware/speech-command-test`: local ESP-SR wake-word/command test.

Flash the streaming firmware:

```bash
cd install/rover_waveshare_audio/share/rover_waveshare_audio/firmware/speech-stream-stt
PORT=/dev/cu.usbmodem11401 ./flash.sh
```

On Raspberry Pi the port is usually `/dev/ttyACM0`.

## Host Dependencies

Install Python dependencies on the computer that runs the ROS node:

```bash
python3 -m pip install -U openai-whisper
```

Whisper also needs PyTorch. On macOS/desktop this is usually installed by the
command above. On Raspberry Pi, install the PyTorch build that matches your OS
and Python version, then verify:

```bash
python3 - <<'PY'
import whisper
import torch
print('whisper ok')
print('torch', torch.__version__)
PY
```

If Whisper fails with `module 'coverage.types' has no attribute 'Tracer'`,
update the unrelated `coverage` package used by Numba during import:

```bash
python3 -m pip install -U 'coverage>=7.6.1'
```

For high quality Russian TTS playback on Raspberry Pi install `ffmpeg` and
Piper:

```bash
sudo apt update
sudo apt install ffmpeg
python3 -m pip install -U piper-tts
```

Then download the recommended Russian Piper voice:

```bash
cd ~/sverk_rover
src/peripherals/rover_waveshare_audio/tools/install_piper_ru_voice.sh
```

The default config uses `tts_engine: piper` with model `ru_RU-irina-medium`.
If Piper is not available, you can still use `espeak-ng` as a simple fallback:

```bash
sudo apt install ffmpeg espeak-ng
```

On macOS the node can also use the built-in `say` command.

## Run

Standalone:

```bash
ros2 launch rover_waveshare_audio waveshare_audio.launch.py
```

From rover bringup:

```bash
ros2 launch rover_bringup robot.launch.py use_waveshare_audio:=true
```

Example override:

```bash
ros2 launch rover_waveshare_audio waveshare_audio.launch.py \
  whisper_model:=small \
  output_topic:=/agent/text
```

Speak a one-shot phrase through the module:

```bash
ros2 service call /voice/say rover_interfaces/srv/SpeakText "{text: 'Привет, проверка динамика'}"
```

## Important Parameters

- `serial_device`: serial device, default `/dev/waveshare_audio`.
- `baudrate`: streaming firmware baud rate, default `2000000`.
- `output_topic`: text output topic, default `/voice/text`.
- `whisper_model`: `tiny`, `base`, `small`, `medium`, `large`, `turbo`, etc.
- `language`: `ru`, `en`, or empty string for Whisper auto-detect.
- `device`: `auto`, `cpu`, `cuda`, `mps`.
- `min_rms`, `start_frames`, `stop_frames`: simple energy VAD tuning.
- `enable_tts`: expose the speech playback service.
- `tts_service_name`: service for playback, default `/voice/say`.
- `tts_engine`: `piper`, `auto`, `say`, or `espeak-ng`.
- `piper_model`: Piper voice name, default `ru_RU-irina-medium`.
- `piper_data_dir`: directory with downloaded Piper voices.
- `tts_voice`: optional voice name for `say`/`espeak-ng`.
- `tts_rate`: speech rate passed to the selected synthesizer.

## Udev

Install the optional udev rule on Raspberry Pi:

```bash
sudo cp install/rover_waveshare_audio/share/rover_waveshare_audio/udev/99-rover-waveshare-audio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then reconnect the board and check:

```bash
ls -l /dev/waveshare_audio
```
