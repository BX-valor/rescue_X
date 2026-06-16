# 庐山派 K230 视觉程序

## YOLO 救援搬运主线

`yolo_sender.py` 是当前任务的主线程序：K230 只做 YOLO11s 识别，并通过串口把
检测结果发送给 ESP32；ESP32 负责目标选择、底盘运动和运输框舵机状态机。

部署文件：

```text
yolo_sender.py
rescue_protocol.py
yolo11s_best_704.kmodel
```

仓库中的模型位于 `model/yolo11s_best_704.kmodel`。复制到 K230 后，请把
`yolo_sender.py` 里的 `MODEL_PATH` 改成板上实际路径，例如：

```python
MODEL_PATH = "/sdcard/yolo11s_best_704.kmodel"
```

默认队伍为红队，目标类别为 `red_ball`、`black_ball`、`yellow_ball`，安全区为
`red_safe_zone`。当前模型为 `yolo11s_best_704.kmodel`，类别顺序必须和
`model/data.yaml` 保持一致。

串口协议为 `0xAA 0xBB + type + frame_id + payload_len + payload + crc16`，
检测框使用 0-1000 归一化中心点和宽高。K230 只需要 TX 接到 ESP32 的视觉
UART RX，并共地。默认 `UART_TX_PIN` 只是占位，请按实车接线修改。

## LAB 颜色视觉原型

该目录使用 CanMV K230 的 `find_blobs` 完成：

- 红、蓝、黄、黑四种小球识别
- 紫色空心安全区方框识别
- 小球面积、长宽比和填充率过滤
- 连续多帧确认
- 判断球心是否位于安全区内

## 运行

将以下文件放在庐山派的同一目录：

```text
color_config.py
color_vision.py
```

在 CanMV IDE K230 中运行 `color_vision.py`。程序默认使用 `320x240`
图像，并通过虚拟显示输出识别框。

串行终端会输出：

```text
ZONE,purple,160,120,180,130
BALL,red,142,176,24,23,0
BALL,yellow,201,168,21,22,1
```

最后一项表示球是否处于紫色安全区内。这个文本输出仅用于调试，后续应换成
带帧头、长度和校验的二进制 UART 协议发送给 ESP32。

## 阈值标定

CanMV `find_blobs` 在 RGB565 图像上使用 LAB 阈值，不是 OpenCV HSV。
两者都是颜色分割，LAB 可以直接使用 K230 的硬件和 OpenMV 接口。

1. 固定最终使用的摄像头、曝光、安装高度和赛场灯光。
2. 在 CanMV IDE 中打开“工具 -> 机器视觉 -> 阈值编辑器”。
3. 分别框选红球、蓝球、黄球、黑球和紫色安全区。
4. 将得到的六元组写入 `color_config.py`。
5. 采集阴影、反光、远近和运动状态下的样本，适当扩大阈值。
6. 若两个颜色互相误检，优先收窄 A/B 范围，不要先扩大面积过滤。

黑球主要依赖低亮度 L 值，容易与阴影混淆。现场应保持曝光稳定，并结合球的
近圆长宽比、尺寸和连续多帧检测。紫色安全区使用大尺寸、低填充率过滤，因此
可以和紫色实心物体区分。

## 首次调试顺序

1. 单独摆放每一种球，完成五组 LAB 阈值标定。
2. 调整 `BALL_MIN_PIXELS` 和 `BALL_MAX_PIXELS`。
3. 在不同距离检查球框长宽比。
4. 放入紫色方框，调整 `SAFE_MIN_WIDTH`、`SAFE_MIN_HEIGHT`。
5. 将球放在框内外，检查输出最后一位是否正确。
6. 最后再接入 ESP32 和运动控制。
