# 真机部署与调试指南

## 1. 需要准备的文件

K230 端复制以下文件到同一目录，例如 `/sdcard/rescue_x/`：

- `K230/check_files.py`
- `K230/yolo_test.py`
- `K230/yolo_sender.py`
- `K230/rescue_protocol.py`
- `model/yolo11s_best_704.kmodel`

ESP32 端烧录：

- `car_control/car_control.ino`

当前主线分工是：K230 只做 YOLO11s 识别和串口发送检测帧；ESP32 负责两轮差速底盘、运输框舵机和自动搬运状态机。

## 2. 上车前必须修改的配置

### K230: `K230/yolo_sender.py`

- `MODEL_PATH`: 改成模型在 K230 上的实际路径。
  - 示例：`MODEL_PATH = "/sdcard/rescue_x/yolo11s_best_704.kmodel"`
- `UART_TX_PIN`: 改成 K230 接到 ESP32 的 TX 引脚。
- `UART_TX_FUNCTION`: 必须和所选 UART 外设匹配，例如 `"UART2_TXD"`。
- `FRAME_WIDTH` / `FRAME_HEIGHT`: 必须和模型输入尺寸一致，当前为 `704 x 704`。
- `CONFIDENCE_THRESHOLD`: 误检多就调高，漏检多就调低，建议先在 `0.30-0.45` 范围内试。
- `DEBUG_RAW_OUTPUTS`: 调试 YOLO11s 输出解析时保持 `True`；稳定后可改为 `False` 减少串口刷屏。
- `MODEL_INPUT_LAYOUT`: 当前默认 `"AI2D_NCHW"`，由 K230 的 `nncase_runtime.ai2d`
  把摄像头 `HWC` 图像转成模型常见的 `1x3x704x704` 输入。不要在 K230 上用
  Python/ulab 手动 `transpose`，704 输入会非常慢甚至卡住。
- `YOLO_OUTPUT_FORMAT`: 当前模型输出为 `(1, 14, 10164)`，即 YOLO11 的 `4 + 10类` 格式，保持 `"YOLO11_4_NC"`。
- `SENSOR_PIXFORMAT`: 默认 `"RGB888"`；如果只有低分误检，可尝试改成 `"RGB565"` 排查输入格式问题。

### ESP32: `car_control/car_control.ino`

- `VISION_UART_RX_PIN`: 改成 ESP32 接 K230 TX 的 RX 引脚。
- `VISION_UART_TX_PIN`: 当前为 `-1`，表示 ESP32 不向 K230 回传；如需双向通信再配置。
- `FRAME_UP_ANGLE` / `FRAME_DOWN_ANGLE`: 实测运输框抬起和放下角度后填写。
- `DIFF_TURN_GAIN`: 转向太猛就调小，转向不足就调大。
- `CAPTURE_CY_THRESHOLD` / `CAPTURE_H_THRESHOLD`: 小球足够近时进入捕获，过早捕获就调高。
- `SAFE_CY_THRESHOLD` / `SAFE_H_THRESHOLD`: 安全区足够近时释放，撞安全区就调低。

## 3. 接线检查

- K230 TX -> ESP32 `VISION_UART_RX_PIN`
- K230 GND -> ESP32 GND
- 串口电平使用 3.3V TTL，不要接 5V。
- ESP32 `Serial2` 已用于 SBUS：GPIO16/17 不要再分配给 K230。
- 两轮差速默认：MOTOR_A 为左轮，MOTOR_B 为右轮。若实车相反，可交换电机线或在代码中交换 `leftMotor` / `rightMotor`。

## 4. 桌面检查

在仓库根目录运行：

```bash
PYTHONPYCACHEPREFIX=/private/tmp python3 -m py_compile K230/rescue_protocol.py K230/yolo_sender.py
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=K230 python3 -c "from rescue_protocol import *; p=pack_detection_frame(1,[{'class_id':2,'score':90,'cx':510,'cy':830,'w':130,'h':150}]); print(p[:2].hex(), crc16_ccitt(p[2:-2])==int.from_bytes(p[-2:],'big'))"
```

ESP32 使用 Arduino IDE 或 PlatformIO 编译。需要安装 ESP32 开发板支持、`ESP32Servo`、`sbus` 等依赖。

## 5. 先单独测试 YOLO

这一步不接 ESP32，只确认 K230 的摄像头、模型加载、推理后处理和显示是否正常。

1. 把 `check_files.py`、`yolo_test.py`、`yolo_sender.py`、`rescue_protocol.py` 和 `yolo11s_best_704.kmodel` 放到 K230 同一目录。
2. 修改 `yolo_sender.py` 中的 `MODEL_PATH`，确保它指向板上的模型文件。
3. 先运行 `check_files.py`，确认终端里显示 `import rescue_protocol: OK` 和 `import yolo_sender: OK`。
4. 再运行 `yolo_test.py`。
5. 将红球、黄球、黑球、蓝球、红色安全区分别放到画面中。
6. 看屏幕是否画出框，并观察串口终端是否输出类似：

```text
YOLO,red_ball,86,512,742,96,112
YOLO,yellow_ball,91,430,688,108,120
YOLO,red_safe_zone,78,505,650,450,260
```

输出字段依次为：类别、置信度、中心点 x、中心点 y、宽、高；坐标和尺寸均为 `0-1000` 归一化值。

如果 `yolo_test.py` 正常，再运行 `yolo_sender.py` 连接 ESP32。若 `yolo_test.py` 不正常，先不要调底盘。

注意：CanMV IDE 有时会以 `<stdin>` 方式运行文件，此时 `sys.path` 不会自动包含
`/sdcard/rescue_x`。请使用新版 `check_files.py`、`yolo_test.py`、`yolo_sender.py`，
它们会主动加入该目录。模型文件名建议保持为 `yolo11s_best_704.kmodel`，不要在
`.kmodel` 前留空格。

实时预览默认开启，使用 CanMV 官方 Display 回传到 IDE：
`Display.init(Display.VIRT, width=704, height=704, to_ide=True)`。
运行 `yolo_test.py` 后，在 CanMV IDE 打开图像/帧缓冲预览窗口即可实时看带检测框的画面。
若正式上车时帧率不够，或 IDE 预览导致卡顿，可把 `DISPLAY_TO_IDE` 改为 `False`。

`yolo_test.py` 默认不保存图片，避免 SD 卡写入造成卡顿。如需保存带检测框的调试图片，
把 `SAVE_DEBUG_IMAGES` 改为 `True`：

```text
/sdcard/rescue_x/debug_frames/frame_000.jpg
/sdcard/rescue_x/debug_frames/frame_001.jpg
```

保存成功时串口会打印 `SAVED,...`。把这些 JPG 从 SD 卡拷回电脑即可查看实际检测画面。

## 6. 分阶段真机调试

1. 架空车轮，只给 ESP32 上电，切手动模式，确认左右轮前进、后退、转向方向正确。
2. 单独测试运输框舵机，找到抬起/放下角度，写入 `FRAME_UP_ANGLE` 和 `FRAME_DOWN_ANGLE`。
3. 只运行 K230 的 `yolo_test.py`，打开显示画面，确认红球、黑球、黄球、红色安全区类别稳定。
4. 接 K230 到 ESP32 串口，打开 ESP32 USB 串口日志，确认出现 `AUTO ... -> ...` 状态切换。
5. 架空车轮自动模式测试：手持红球/黄球/黑球移动，确认车轮会搜索、对准、接近。
6. 落地单球测试：先只放一个红球和红安全区，低速验证捕获、找安全区、释放。
7. 多球测试：加入黄球、黑球、蓝球，确认黄/黑优先，蓝球不主动搬运。

## 7. 常见问题

- `yolo_test.py` 启动就报模型错误：确认 `MODEL_PATH`、文件名、SD 卡路径完全正确。
- 报 `no module named rescue_protocol` 或类似错误：确认文件名必须是英文 `rescue_protocol.py`，不是 `rescue_protocal.py`、`rescue_protocol.py.txt` 或 CanMV 另存出来的带后缀副本；先运行 `check_files.py` 看板上真实文件名。
- 卡在 `TENSOR transpose`：使用新版 `yolo_sender.py`，默认 `MODEL_INPUT_LAYOUT = "AI2D_NCHW"`，
  让 AI2D 完成 `HWC -> NCHW`。运行时应看到 `AI2D init OK`、`AI2D run OK`，然后进入
  `KPU run`。
- 报 `ai2d_format has no attribute NHWC_FMT`：使用新版 `yolo_sender.py`，它会自动兼容
  `NHWC_FMT`、`RGB_packed` 等不同固件的 AI2D 格式名；若仍报错，把串口中的
  `AI2D available formats=...` 发回来。
- 报 `function takes 1 positional arguments but 2 were given` 且堆栈指向 `transpose`：说明板上还是旧文件；
  重新复制新版 `yolo_sender.py` 和 `yolo_test.py` 到 `/sdcard/rescue_x/`。
- 串口输出 `YOLO,purple_boundary,...`：说明模型识别到了边界类。ESP32 自动策略会忽略该类，只使用红/黑/黄球和红色安全区。
- 只有黄球却 `TOP` 全是低分 `blue_safe_zone`：先看 `CLASS_TOP,yellow_ball,...` 分数；如果黄球分数也很低，尝试切换 `SENSOR_PIXFORMAT`，并检查模型训练类别顺序和实物光照/距离。
- 报 `current format not support save function`：实时预览不需要保存图片，保持 `SAVE_DEBUG_IMAGES = False`；若必须保存，使用新版 `yolo_test.py`，它会先尝试转为 RGB565 再保存。
- 屏幕有画面但永远 `YOLO none`：先降低 `CONFIDENCE_THRESHOLD` 到 `0.25`；仍无检测则检查模型输入尺寸和类别输出格式。
- 车一直原地转：检查 K230 是否发送检测帧、ESP32 `VISION_UART_RX_PIN` 是否接对、GND 是否共地。
- 检测正常但车转反：交换左右轮，或调换 `target_y + turn` / `target_y - turn` 的符号。
- 一靠近球就提前放框：增大 `CAPTURE_CY_THRESHOLD` 或 `CAPTURE_H_THRESHOLD`。
- 撞到安全区才释放：降低 `SAFE_CY_THRESHOLD` 或 `SAFE_H_THRESHOLD`。
- 误搬蓝球：确认模型类别顺序与 `model/data.yaml` 完全一致，尤其是 `red_ball=1`、`yellow_ball=2`、`blue_ball=3`、`black_ball=4`、`red_safe_zone=5`。
- K230 程序启动报模型错误：确认 `MODEL_PATH` 存在，并确认 `yolo11s_best_704.kmodel` 是 K230 可运行的 kmodel。

## 8. 上场前检查清单

- 红队固定策略已确认，目标为红、黑、黄球。
- K230 模型路径、UART TX 引脚已改。
- ESP32 视觉 UART RX 引脚已改。
- 两轮方向、SBUS 手动接管、运输框角度均已实测。
- 视觉中断 500ms 后会停车。
- `model/` 下的大文件变动不要误提交，除非确认需要更新模型版本。
