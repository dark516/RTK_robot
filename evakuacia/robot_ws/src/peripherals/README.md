# Peripheral packages

This folder groups ROS packages that communicate with external rover hardware.
The folder itself is not a ROS package; colcon discovers the packages below it recursively.

Packages:

- `rover_base_driver` - motor controller communication and encoder feedback.
- `rover_camera` - USB camera driver.
- `rover_device_manager` - serial device discovery and persistent device setup.
- `rover_imu` - Yahboom IMU driver and normalization tools.
- `rover_led_strip` - addressable LED strip driver.
- `rover_octoliner` - Amperka Octoliner line sensor driver.
- `rover_waveshare_audio` - Waveshare ESP32-S3-AUDIO-Board audio streaming, Whisper speech-to-text, and text-to-speech playback bridge.
- `sllidar_ros2` - SLLIDAR/RPLIDAR ROS 2 driver.
