"""K230 YOLO detection sender for the rescue car.

Copy `model/yolo11s_best_704.kmodel` to the board path configured by
MODEL_PATH, then run this file on the Lushan Pi K230. The ESP32 receives
normalized detections and owns the motion state machine.
"""

import gc
import os
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor

from rescue_protocol import (
    CLASS_NAMES,
    TEAM_RED,
    normalized_center_box,
    pack_detection_frame,
)


MODEL_PATH = "/sdcard/yolo11s_best_704.kmodel"
FRAME_WIDTH = 704
FRAME_HEIGHT = 704
MODEL_INPUT_LAYOUT = "NCHW"

CONFIDENCE_THRESHOLD = 0.35
NMS_THRESHOLD = 0.45
MAX_DETECTIONS = 12
SEND_INTERVAL_MS = 100
SHOW_IMAGE = True

UART_ID = 2
UART_TX_PIN = 11
UART_TX_FUNCTION = "UART2_TXD"
UART_BAUD = 115200

DRAW_COLORS = {
    "red_ball": (255, 0, 0),
    "blue_ball": (0, 80, 255),
    "yellow_ball": (255, 220, 0),
    "black_ball": (255, 255, 255),
    "red_safe_zone": (255, 60, 60),
    "blue_safe_zone": (60, 120, 255),
}


class Detection:
    def __init__(self, class_id, score, cx, cy, w, h):
        self.class_id = int(class_id)
        self.score = int(score)
        self.cx = int(cx)
        self.cy = int(cy)
        self.w = int(w)
        self.h = int(h)


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(now, before):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, before)
    return now - before


def setup_uart():
    from machine import FPIOA, UART

    fpioa = FPIOA()
    tx_function = getattr(FPIOA, UART_TX_FUNCTION, UART_TX_FUNCTION)
    fpioa.set_function(UART_TX_PIN, tx_function)

    uart_id = getattr(UART, "UART%d" % UART_ID, UART_ID)
    try:
        return UART(
            uart_id,
            baudrate=UART_BAUD,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )
    except TypeError:
        return UART(uart_id, UART_BAUD)


def iou(box_a, box_b):
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - inter_area
    if union <= 0:
        return 0
    return inter_area / union


def nms(detections, threshold):
    detections.sort(key=lambda item: item.score, reverse=True)
    kept = []

    for det in detections:
        cx = det.cx
        cy = det.cy
        half_w = det.w // 2
        half_h = det.h // 2
        det_box = (cx - half_w, cy - half_h, cx + half_w, cy + half_h)
        suppressed = False

        for existing in kept:
            if existing.class_id != det.class_id:
                continue
            ex_box = (
                existing.cx - existing.w // 2,
                existing.cy - existing.h // 2,
                existing.cx + existing.w // 2,
                existing.cy + existing.h // 2,
            )
            if iou(det_box, ex_box) > threshold:
                suppressed = True
                break

        if not suppressed:
            kept.append(det)
            if len(kept) >= MAX_DETECTIONS:
                break

    return kept


class K230YoloDetector:
    def __init__(self, model_path):
        import nncase_runtime as nn
        import ulab.numpy as np

        self.nn = nn
        self.np = np
        self.kpu = nn.kpu()
        self.kpu.load_kmodel(model_path)

    def image_to_tensor(self, img):
        if hasattr(img, "to_numpy_ref"):
            arr = img.to_numpy_ref()
        elif hasattr(img, "to_numpy"):
            arr = img.to_numpy()
        else:
            raise RuntimeError("Image object does not expose to_numpy_ref/to_numpy")

        shape = arr.shape
        if MODEL_INPUT_LAYOUT == "NCHW":
            if len(shape) == 3 and shape[0] == 3:
                arr = arr.reshape((1, 3, shape[1], shape[2]))
            elif len(shape) == 3:
                arr = arr.transpose((2, 0, 1)).reshape((1, 3, shape[0], shape[1]))
        else:
            if len(shape) == 3:
                arr = arr.reshape((1, shape[0], shape[1], shape[2]))

        return self.nn.from_numpy(arr)

    def run_outputs(self, img):
        input_tensor = self.image_to_tensor(img)
        self.kpu.set_input_tensor(0, input_tensor)
        self.kpu.run()

        outputs = []
        output_count = self.kpu.outputs_size()
        for index in range(output_count):
            outputs.append(self.kpu.get_output_tensor(index).to_numpy())
        return outputs

    def detect(self, img):
        outputs = self.run_outputs(img)
        if not outputs:
            return []
        return self.postprocess(outputs[0])

    def postprocess(self, output):
        rows = self.output_rows(output)
        detections = []
        class_count = len(CLASS_NAMES)

        for row in rows:
            row_len = len(row)

            if row_len == 6:
                det = self.postprocess_six_value_row(row)
                if det:
                    detections.append(det)
                continue

            if row_len < 4 + class_count:
                continue

            best_class = 0
            best_score = 0
            for class_id in range(class_count):
                score = row[4 + class_id]
                if score > best_score:
                    best_score = score
                    best_class = class_id

            if best_score < CONFIDENCE_THRESHOLD:
                continue

            cx, cy, w, h = self.scale_box(row[0], row[1], row[2], row[3])
            detections.append(
                Detection(best_class, int(best_score * 100), cx, cy, w, h)
            )

        return nms(detections, NMS_THRESHOLD)

    def postprocess_six_value_row(self, row):
        class_count = len(CLASS_NAMES)

        # Common postprocessed form: x1, y1, x2, y2, score, class_id.
        if row[4] <= 1.5 and 0 <= int(row[5]) < class_count:
            score = row[4]
            if score < CONFIDENCE_THRESHOLD:
                return None
            cx = (row[0] + row[2]) / 2
            cy = (row[1] + row[3]) / 2
            w = row[2] - row[0]
            h = row[3] - row[1]
            cx, cy, w, h = self.scale_box(cx, cy, w, h)
            return Detection(int(row[5]), int(score * 100), cx, cy, w, h)

        # Project-local form: class_id, score_0_100, cx, cy, w, h.
        if 0 <= int(row[0]) < class_count:
            score = row[1] / 100 if row[1] > 1.5 else row[1]
            if score < CONFIDENCE_THRESHOLD:
                return None
            cx, cy, w, h = self.scale_box(row[2], row[3], row[4], row[5])
            return Detection(int(row[0]), int(score * 100), cx, cy, w, h)

        return None

    def output_rows(self, output):
        shape = output.shape
        class_columns = 4 + len(CLASS_NAMES)

        if len(shape) == 3:
            output = output[0]
            shape = output.shape

        if len(shape) == 2 and shape[0] == class_columns and shape[1] != class_columns:
            output = output.transpose()

        return output

    def scale_box(self, cx, cy, w, h):
        # Support either normalized boxes or model-input pixel boxes.
        if max(cx, cy, w, h) <= 1.5:
            return (
                int(cx * 1000),
                int(cy * 1000),
                int(w * 1000),
                int(h * 1000),
            )
        return normalized_center_box(cx, cy, w, h, FRAME_WIDTH, FRAME_HEIGHT)


def draw_detections(img, detections):
    for det in detections:
        if det.class_id >= len(CLASS_NAMES):
            continue

        name = CLASS_NAMES[det.class_id]
        color = DRAW_COLORS.get(name, (255, 255, 255))
        x = int((det.cx - det.w / 2) * FRAME_WIDTH / 1000)
        y = int((det.cy - det.h / 2) * FRAME_HEIGHT / 1000)
        w = int(det.w * FRAME_WIDTH / 1000)
        h = int(det.h * FRAME_HEIGHT / 1000)

        if hasattr(img, "draw_rectangle"):
            img.draw_rectangle((x, y, w, h), color=color, thickness=2)
        if hasattr(img, "draw_string"):
            img.draw_string(x, max(0, y - 14), "%s %d" % (name, det.score), color=color)


def main():
    sensor = None
    frame_id = 0
    last_send = 0

    try:
        uart = setup_uart()
        detector = K230YoloDetector(MODEL_PATH)

        sensor = Sensor(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB888)

        if SHOW_IMAGE:
            Display.init(Display.VIRT, width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=30)

        MediaManager.init()
        sensor.run()

        while True:
            os.exitpoint()
            img = sensor.snapshot()
            detections = detector.detect(img)

            if SHOW_IMAGE:
                draw_detections(img, detections)
                Display.show_image(img)

            now = ticks_ms()
            if ticks_diff(now, last_send) >= SEND_INTERVAL_MS:
                packet = pack_detection_frame(frame_id, detections, TEAM_RED)
                uart.write(packet)
                frame_id = (frame_id + 1) & 0xFF
                last_send = now

            gc.collect()

    except KeyboardInterrupt:
        print("Stopped by user")
    except BaseException as error:
        print("YOLO sender error: %s" % error)
        raise
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        if SHOW_IMAGE:
            Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
