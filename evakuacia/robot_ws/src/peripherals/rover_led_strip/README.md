# rover_led_strip

ROS 2 package for controlling an addressable LED strip from the rover and from `rover_web`.

## Raspberry Pi 5 Python dependencies

On the rover install the Pi 5 NeoPixel stack once:

```bash
python3 -m pip install --upgrade Adafruit-Blinka adafruit-circuitpython-pixelbuf Adafruit-Blinka-Raspberry-Pi5-Neopixel
```

## Wiring note

The current config defaults to `GPIO18`, which is the recommended data pin for this project.

Avoid using `GPIO2` for the LED data line when Octoliner or other I2C devices are connected. `GPIO2` is the default I2C SDA pin on Raspberry Pi, so it can conflict with I2C traffic.
