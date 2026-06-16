# rescue_X
## 工创赛救援小车
### car_control
car_control包含救援小车的控制代码，救援小车使用esp32作为主控。当前底盘为
左右两轮差速驱动，前方使用万向轮支撑。
### MaixCam
MaixCam包含MaixCam的小车识别小球与安全区的代码，以及小车整体的控制逻辑。uartCommand.py和control_servo.py请放置在MaixCam的/root/models/文件下
### model
model包含MaixCam做yolo识别需要使用的模型文件。模型文件请放在/model/scripts/文件下。

### K230
K230包含立创庐山派K230视觉程序。当前主线为`yolo_sender.py`：使用
`model/yolo11s_best_704.kmodel`识别救援目标和安全区，并通过串口向ESP32
发送检测帧。`yolo_test.py`用于单独测试YOLO检测与IDE预览（无需连接ESP32）。
`color_vision.py`保留为LAB颜色视觉调试原型。
