# ESP32-S3 Audio Serial STT Bridge

This project turns the Waveshare `ESP32-S3-AUDIO-Board` into a USB serial audio front-end.

The board:

- captures microphone audio,
- mixes the two microphone channels down to `16 kHz mono s16le`,
- sends framed PCM packets over `USB Serial/JTAG`.
- only streams audio while the center user button is held, plus a `2s` grace period after release.

The host scripts:

- reads the PCM frames,
- detects utterances using a simple energy-based VAD,
- transcribes each utterance with `Whisper`,
- prints text to `stdout`,
- can also append text to a file or forward it to a TCP socket or another serial/UART port.
- can also run a simple voice-assistant loop and send spoken replies back to the board speaker.

## Firmware

Flash the board from the source workspace:

```bash
cd ~/sverk_rover/src/peripherals/rover_waveshare_audio/firmware/speech-stream-stt
./flash.sh
```

Or from an installed workspace:

```bash
cd ~/sverk_rover/install/rover_waveshare_audio/share/rover_waveshare_audio/firmware/speech-stream-stt
./flash.sh
```

## Host transcription

Run live transcription:

```bash
cd ~/sverk_rover/src/peripherals/rover_waveshare_audio/firmware/speech-stream-stt
./host/run_transcriber.sh --language ru
```

## Voice assistant loop

Run a simple `speech -> text -> reply -> speech` loop:

```bash
cd ~/sverk_rover/src/peripherals/rover_waveshare_audio/firmware/speech-stream-stt
./host/run_voice_agent.sh --language ru --model tiny
```

By default the spoken reply is a simple template:

```text
Я услышал: <ваша фраза>
```

You can plug in any external agent process. The recognized text is passed to the command on `stdin`, and the command reply is read from `stdout`:

```bash
./host/run_voice_agent.sh --language ru --agent-command "python3 my_agent.py"
```

Useful voice-agent options:

```bash
./host/run_voice_agent.sh --language ru --tts-voice Milena
./host/run_voice_agent.sh --language ru --append-file conversation.txt
./host/run_voice_agent.sh --language ru --jsonl-file conversation.jsonl
./host/run_voice_agent.sh --list-voices
./host/run_voice_agent.sh --speak-text "Привет, проверка динамика"
```

Push-to-talk behavior:

- hold the center user button on the board to speak
- after release, the board keeps streaming for about `2` more seconds
- LED states: white = idle, blinking green = listening

Pick a specific Whisper model for testing:

```bash
./host/run_transcriber.sh --language ru --model tiny
./host/run_transcriber.sh --language ru --model base
./host/run_transcriber.sh --language ru --model small
./host/run_transcriber.sh --language ru --model medium
./host/run_transcriber.sh --language ru --model large
./host/run_transcriber.sh --language ru --model turbo
```

Show all supported model names:

```bash
./host/run_transcriber.sh --list-models
```

Useful options:

```bash
./host/run_transcriber.sh --language ru --append-file transcripts.txt
./host/run_transcriber.sh --language ru --jsonl-file transcripts.jsonl
./host/run_transcriber.sh --language ru --tcp 127.0.0.1:9000
./host/run_transcriber.sh --language ru --out-serial /dev/cu.usbserial-0001 --out-serial-baud 115200
./host/run_transcriber.sh --language ru --save-wavs-dir utterances
```

If the board gets another serial port:

```bash
./host/run_transcriber.sh --port /dev/cu.usbmodem11401
```
