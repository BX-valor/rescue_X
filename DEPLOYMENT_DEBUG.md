# 真机部署与调试指南

## 1. 需要准备的文件

K230 端复制以下文件到同一目录，例如 `/sdcard/rescue_x/`：

- `K230/yolo_sender.py`
- `K230/rescue_protocol.py`
- `model/yolo11s_best_704.kmodel`

ESP32 端烧录：

- `car_control/car_control.ino`

当前主线分工是：K230 只做 YOLO 识别和串口发送检测帧；ESP32 负责两轮差速底盘、运输框舵机和自动搬运状态机。

## 2. 上车前必须修改的配置

### K230: `K230/yolo_sender.py`

- `MODEL_PATH`: 改成模型在 K230 上的实际路径。
  - 示例：`MODEL_PATH = "/sdcard/rescue_x/yolo11s_best_704.kmodel"`
- `UART_TX_PIN`: 改成 K230 接到 ESP32 的 TX 引脚。
- `UART_TX_FUNCTION`: 必须和所选 UART 外设匹配，例如 `"UART2_TXD"`。
- `FRAME_WIDTH` / `FRAME_HEIGHT`: 必须和模型输入尺寸一致，当前为 `704 x 704`。
- `CONFIDENCE_THRESHOLD`: 误检多就调高，漏检多就调低，建议先在 `0.30-0.45` 范围内试。

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

## 5. 分阶段真机调试

1. 架空车轮，只给 ESP32 上电，切手动模式，确认左右轮前进、后退、转向方向正确。
2. 单独测试运输框舵机，找到抬起/放下角度，写入 `FRAME_UP_ANGLE` 和 `FRAME_DOWN_ANGLE`。
3. 只运行 K230，打开显示画面，确认红球、黑球、黄球、红色安全区类别稳定。
4. 接 K230 到 ESP32 串口，打开 ESP32 USB 串口日志，确认出现 `AUTO ... -> ...` 状态切换。
5. 架空车轮自动模式测试：手持红球/黄球/黑球移动，确认车轮会搜索、对准、接近。
6. 落地单球测试：先只放一个红球和红安全区，低速验证捕获、找安全区、释放。
7. 多球测试：加入黄球、黑球、蓝球，确认黄/黑优先，蓝球不主动搬运。

## 6. 常见问题

- 车一直原地转：检查 K230 是否发送检测帧、ESP32 `VISION_UART_RX_PIN` 是否接对、GND 是否共地。
- 检测正常但车转反：交换左右轮，或调换 `target_y + turn` / `target_y - turn` 的符号。
- 一靠近球就提前放框：增大 `CAPTURE_CY_THRESHOLD` 或 `CAPTURE_H_THRESHOLD`。
- 撞到安全区才释放：降低 `SAFE_CY_THRESHOLD` 或 `SAFE_H_THRESHOLD`。
- 误搬蓝球：确认模型类别顺序与 `model/data.yaml` 完全一致，尤其是 `red_ball=1`、`yellow_ball=2`、`blue_ball=3`、`black_ball=4`、`red_safe_zone=5`。
- K230 程序启动报模型错误：确认 `MODEL_PATH` 存在，并确认 `yolo11s_best_704.kmodel` 是 K230 可运行的 kmodel。

## 7. 上场前检查清单

- 红队固定策略已确认，目标为红、黑、黄球。
- K230 模型路径、UART TX 引脚已改。
- ESP32 视觉 UART RX 引脚已改。
- 两轮方向、SBUS 手动接管、运输框角度均已实测。
- 视觉中断 500ms 后会停车。
- `model/` 下的大文件变动不要误提交，除非确认需要更新模型版本。
