# image processor node for RTK gorod task.

running without debugging log (camera window)
```bash
ros2 run image_processor image_processor
```
with debug log
```bash
ros2 run image_processor image_processor --ros-args -p show_debug:=True
```
the yolo weights are located in the ```/resource/best1.pt``` folder
