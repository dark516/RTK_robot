# ESP32-S3 Audio Speech Test

Minimal test firmware for the Waveshare `ESP32-S3-AUDIO-Board`.

What it does:

- Wake word: `hi esp`
- Recognizes 4 built-in English commands from the official Waveshare `esp_sr_02` example
- Shows command results on the onboard RGB ring
- Prints status and recognized commands to the USB serial console

Recognized commands:

1. `turn on the backlight` -> RGB ring turns white
2. `turn off the backlight` -> RGB ring turns off
3. `backlight is brightest` -> RGB ring turns green
4. `backlight is darkest` -> RGB ring turns red

Quick start:

```bash
cd "/Users/urijgolysev/Documents/sniz/voice test/speech-command-test"
./flash.sh
./monitor.sh
```

If the board gets a different serial port, override it:

```bash
PORT=/dev/cu.usbmodem11401 ./flash.sh
PORT=/dev/cu.usbmodem11401 ./monitor.sh
```
