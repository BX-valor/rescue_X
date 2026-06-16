"""K230 YOLO detection sender for the rescue car.

Copy `model/yolo11s_best_704.kmodel` to the board path configured by
MODEL_PATH, then run this file on the Lushan Pi K230. The ESP32 receives
normalized detections and owns the motion state machine.
"""

import gc
import os
import sys
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor

DEPLOY_DIR = "/sdcard/rescue_x"

try:
    SCRIPT_DIR = __file__.rsplit("/", 1)[0]
    if SCRIPT_DIR and SCRIPT_DIR not in sys.path:
        sys.path.append(SCRIPT_DIR)
except Exception:
    pass

if DEPLOY_DIR not in sys.path:
    sys.path.append(DEPLOY_DIR)

from rescue_protocol import (
    CLASS_NAMES,
    TEAM_RED,
    normalized_center_box,
    pack_detection_frame,
)


MODEL_PATH = "/sdcard/rescue_x/yolo11s_best_704.kmodel"
MODEL_PATH_FALLBACKS = (
    "/sdcard/rescue_x/yolo11s_best_704 .kmodel",
    "/sdcard/yolo11s_best_704.kmodel",
)
FRAME_WIDTH = 704
FRAME_HEIGHT = 704
MODEL_INPUT_LAYOUT = "AI2D_NCHW"
SENSOR_PIXFORMAT = "RGB888"

CONFIDENCE_THRESHOLD = 0.25
NMS_THRESHOLD = 0.45
MAX_DETECTIONS = 12
SEND_INTERVAL_MS = 100
SHOW_IMAGE = False
DISPLAY_TO_IDE = False
DISPLAY_WIDTH = 352
DISPLAY_HEIGHT = 352
DEBUG_RAW_OUTPUTS = False
DEBUG_RAW_FRAMES = 3
DEBUG_TOP_CANDIDATES = False
DEBUG_TOP_EVERY_FRAME = False
DEBUG_CLASS_CANDIDATES = False
DEBUG_INFERENCE_STEPS = False
YOLO_OUTPUT_FORMAT = "YOLO11_4_NC"

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


def sensor_pixformat_value(Sensor):
    if SENSOR_PIXFORMAT == "RGB565":
        return Sensor.RGB565
    return Sensor.RGB888


def path_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def resolve_model_path():
    if path_exists(MODEL_PATH):
        return MODEL_PATH

    for path in MODEL_PATH_FALLBACKS:
        if path_exists(path):
            print("Using fallback model path: %s" % path)
            return path

    return MODEL_PATH


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
        self.debug_frames_left = DEBUG_RAW_FRAMES
        self.ai2d = None
        self.ai2d_builder = None
        self.ai2d_output_tensor = None
        if MODEL_INPUT_LAYOUT == "AI2D_NCHW":
            self.init_ai2d()

    def init_ai2d(self):
        if DEBUG_INFERENCE_STEPS:
            print("AI2D init")
        self.ai2d = self.nn.ai2d()
        src_format = self.ai2d_format(
            ("NHWC_FMT", "NHWC", "RGB_packed", "RGB_PACKED"),
            "input",
        )
        dst_format = self.ai2d_format(
            ("NCHW_FMT", "NCHW"),
            "output",
        )
        self.ai2d.set_dtype(
            src_format,
            dst_format,
            self.np.uint8,
            self.np.uint8,
        )
        self.ai2d_builder = self.ai2d.build(
            [1, FRAME_HEIGHT, FRAME_WIDTH, 3],
            [1, 3, FRAME_HEIGHT, FRAME_WIDTH],
        )
        output_data = self.np.ones(
            (1, 3, FRAME_HEIGHT, FRAME_WIDTH),
            dtype=self.np.uint8,
        )
        self.ai2d_output_tensor = self.nn.from_numpy(output_data)
        if DEBUG_INFERENCE_STEPS:
            print("AI2D init OK")

    def ai2d_format(self, names, label):
        for name in names:
            if hasattr(self.nn.ai2d_format, name):
                value = getattr(self.nn.ai2d_format, name)
                if DEBUG_INFERENCE_STEPS:
                    print("AI2D %s format=%s" % (label, name))
                return value

        try:
            available = dir(self.nn.ai2d_format)
        except Exception:
            available = []
        print("AI2D available formats=%s" % available)
        raise RuntimeError("No supported AI2D %s format found" % label)

    def image_to_tensor(self, img):
        if DEBUG_INFERENCE_STEPS:
            print("TENSOR get image array")
        if hasattr(img, "to_numpy_ref"):
            arr = img.to_numpy_ref()
        elif hasattr(img, "to_numpy"):
            arr = img.to_numpy()
        else:
            raise RuntimeError("Image object does not expose to_numpy_ref/to_numpy")

        shape = arr.shape
        if DEBUG_INFERENCE_STEPS:
            print("TENSOR input shape=%s" % (shape,))
        if MODEL_INPUT_LAYOUT == "AI2D_NCHW":
            if len(shape) != 3:
                raise RuntimeError("AI2D_NCHW expects HWC image data")
            if DEBUG_INFERENCE_STEPS:
                print("AI2D reshape input")
            input_data = arr.reshape((1, shape[0], shape[1], shape[2]))
            input_tensor = self.nn.from_numpy(input_data)
            if DEBUG_INFERENCE_STEPS:
                print("AI2D run")
            self.ai2d_builder.run(input_tensor, self.ai2d_output_tensor)
            if DEBUG_INFERENCE_STEPS:
                print("AI2D run OK")
            return self.ai2d_output_tensor

        if MODEL_INPUT_LAYOUT in ("AUTO", "NHWC"):
            if len(shape) == 3:
                arr = arr.reshape((1, shape[0], shape[1], shape[2]))
            elif len(shape) == 4:
                arr = arr
        elif MODEL_INPUT_LAYOUT == "NCHW":
            if len(shape) == 3 and shape[0] == 3:
                arr = arr.reshape((1, 3, shape[1], shape[2]))
            elif len(shape) == 3:
                # ulab on CanMV does not support transpose(axis_tuple).  The
                # no-arg transpose reverses HWC to CWH; with square model input
                # this gives the required channel-first memory layout.
                if DEBUG_INFERENCE_STEPS:
                    print("TENSOR transpose")
                arr = arr.transpose().reshape((1, shape[2], shape[1], shape[0]))
        else:
            if len(shape) == 3:
                arr = arr.reshape((1, shape[0], shape[1], shape[2]))

        if DEBUG_INFERENCE_STEPS:
            print("TENSOR output shape=%s" % (arr.shape,))
            print("TENSOR from_numpy")
        return self.nn.from_numpy(arr)

    def run_outputs(self, img):
        if DEBUG_INFERENCE_STEPS:
            print("KPU make input tensor")
        input_tensor = self.image_to_tensor(img)
        if DEBUG_INFERENCE_STEPS:
            print("KPU set input")
        self.kpu.set_input_tensor(0, input_tensor)
        if DEBUG_INFERENCE_STEPS:
            print("KPU run")
        self.kpu.run()
        if DEBUG_INFERENCE_STEPS:
            print("KPU run OK")

        outputs = []
        output_count = self.kpu.outputs_size()
        if DEBUG_INFERENCE_STEPS:
            print("KPU outputs=%d" % output_count)
        for index in range(output_count):
            if DEBUG_INFERENCE_STEPS:
                print("KPU get output %d" % index)
            outputs.append(self.kpu.get_output_tensor(index).to_numpy())
        if DEBUG_INFERENCE_STEPS:
            print("KPU outputs OK")
        return outputs

    def detect(self, img):
        outputs = self.run_outputs(img)
        if not outputs:
            return []
        self.print_raw_outputs(outputs)
        return self.postprocess(outputs[0])

    def print_raw_outputs(self, outputs):
        if not DEBUG_RAW_OUTPUTS or self.debug_frames_left <= 0:
            return

        print("RAW_OUTPUTS,count=%d" % len(outputs))
        for index, output in enumerate(outputs):
            print("RAW,%d,shape=%s" % (index, output.shape))
            rows = self.output_rows(output)
            printed = 0
            for row in rows:
                print("RAW_ROW,%d,%s" % (index, self.short_row(row)))
                printed += 1
                if printed >= 3:
                    break
        self.debug_frames_left -= 1

    def short_row(self, row):
        values = []
        limit = min(len(row), 16)
        for i in range(limit):
            values.append("%.3f" % row[i])
        return ",".join(values)

    def postprocess(self, output):
        rows = self.output_rows(output)
        detections = []
        class_count = len(CLASS_NAMES)
        class_max_scores = []
        for _ in range(class_count):
            class_max_scores.append(-999)
        top_candidates = []
        class_candidates = []
        for _ in range(class_count):
            class_candidates.append(None)

        for row in rows:
            row_len = len(row)

            if row_len == 4 + class_count and YOLO_OUTPUT_FORMAT == "YOLO11_4_NC":
                det = self.postprocess_yolo11_row(
                    row,
                    class_max_scores,
                    top_candidates,
                    class_candidates,
                )
                if det:
                    detections.append(det)
                continue

            if row_len == 6:
                det = self.postprocess_six_value_row(row)
                if det:
                    detections.append(det)
                continue

            if row_len < 4 + class_count:
                continue

            if row_len >= 5 + class_count:
                det = self.postprocess_objectness_row(row)
                if det:
                    detections.append(det)
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

        self.print_class_max_scores(class_max_scores)
        self.print_top_candidates(top_candidates)
        self.print_class_candidates(class_candidates)
        return nms(detections, NMS_THRESHOLD)

    def postprocess_yolo11_row(
        self,
        row,
        class_max_scores,
        top_candidates,
        class_candidates,
    ):
        class_count = len(CLASS_NAMES)
        best_class = 0
        best_score = row[4]

        for class_id in range(class_count):
            score = row[4 + class_id]
            if score > class_max_scores[class_id]:
                class_max_scores[class_id] = score
                cx, cy, w, h = self.scale_box(row[0], row[1], row[2], row[3])
                class_candidates[class_id] = (score, cx, cy, w, h)
            if score > best_score:
                best_score = score
                best_class = class_id

        self.add_top_candidate(top_candidates, row, best_class, best_score)

        if best_score < CONFIDENCE_THRESHOLD:
            return None

        cx, cy, w, h = self.scale_box(row[0], row[1], row[2], row[3])
        if not self.valid_box(best_class, w, h):
            return None
        return Detection(best_class, int(best_score * 100), cx, cy, w, h)

    def print_class_max_scores(self, class_max_scores):
        if not DEBUG_RAW_OUTPUTS or self.debug_frames_left <= 0:
            return

        parts = []
        for class_id, score in enumerate(class_max_scores):
            parts.append("%s=%.3f" % (CLASS_NAMES[class_id], score))
        print("CLASS_MAX,%s" % ",".join(parts))

    def add_top_candidate(self, top_candidates, row, class_id, score):
        cx, cy, w, h = self.scale_box(row[0], row[1], row[2], row[3])
        item = (score, class_id, cx, cy, w, h)

        top_candidates.append(item)
        top_candidates.sort(key=lambda value: value[0], reverse=True)
        while len(top_candidates) > 5:
            top_candidates.pop()

    def print_top_candidates(self, top_candidates):
        if not DEBUG_TOP_CANDIDATES:
            return
        if not DEBUG_TOP_EVERY_FRAME and self.debug_frames_left <= 0:
            return

        for score, class_id, cx, cy, w, h in top_candidates:
            print(
                "TOP,%s,%.3f,%d,%d,%d,%d"
                % (CLASS_NAMES[class_id], score, cx, cy, w, h)
            )

    def print_class_candidates(self, class_candidates):
        if not DEBUG_CLASS_CANDIDATES:
            return

        for class_id in (1, 2, 3, 4, 5, 6):
            item = class_candidates[class_id]
            if not item:
                continue
            score, cx, cy, w, h = item
            print(
                "CLASS_TOP,%s,%.3f,%d,%d,%d,%d"
                % (CLASS_NAMES[class_id], score, cx, cy, w, h)
            )

    def postprocess_objectness_row(self, row):
        class_count = len(CLASS_NAMES)
        obj_score = row[4]

        best_class = 0
        best_class_score = 0
        for class_id in range(class_count):
            score = row[5 + class_id]
            if score > best_class_score:
                best_class_score = score
                best_class = class_id

        final_score = obj_score * best_class_score
        if final_score < CONFIDENCE_THRESHOLD:
            return None

        cx, cy, w, h = self.scale_box(row[0], row[1], row[2], row[3])
        if not self.valid_box(best_class, w, h):
            return None
        return Detection(best_class, int(final_score * 100), cx, cy, w, h)

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
            if not self.valid_box(int(row[5]), w, h):
                return None
            return Detection(int(row[5]), int(score * 100), cx, cy, w, h)

        # Project-local form: class_id, score_0_100, cx, cy, w, h.
        if 0 <= int(row[0]) < class_count:
            score = row[1] / 100 if row[1] > 1.5 else row[1]
            if score < CONFIDENCE_THRESHOLD:
                return None
            cx, cy, w, h = self.scale_box(row[2], row[3], row[4], row[5])
            if not self.valid_box(int(row[0]), w, h):
                return None
            return Detection(int(row[0]), int(score * 100), cx, cy, w, h)

        return None

    def output_rows(self, output):
        shape = output.shape
        class_columns = 4 + len(CLASS_NAMES)
        objectness_columns = 5 + len(CLASS_NAMES)

        if len(shape) == 3:
            output = output[0]
            shape = output.shape

        if (
            len(shape) == 2
            and shape[0] in (6, class_columns, objectness_columns)
            and shape[1] not in (6, class_columns, objectness_columns)
        ):
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

    def valid_box(self, class_id, w, h):
        if w <= 0 or h <= 0:
            return False

        # Ball detections should not cover most of the frame.
        if class_id in (1, 2, 3, 4) and (w > 450 or h > 450):
            return False

        # Safe zones can be large, but a full-frame safe zone is usually a
        # postprocess mismatch during debugging.
        if class_id in (5, 6) and (w > 950 or h > 950):
            return False

        return True


def _image_size(img):
    if hasattr(img, "width") and hasattr(img, "height"):
        try:
            return (img.width(), img.height())
        except Exception:
            pass
    if hasattr(img, "shape"):
        shape = img.shape
        if len(shape) >= 2:
            return (shape[1], shape[0])
    return (None, None)


def resize_for_display(img):
    """Return a smaller copy of img for IDE preview.

    The YOLO model still runs on the full 704x704 frame, but the IDE stream
    is reduced to save bandwidth and keep the preview responsive.
    """
    if DISPLAY_WIDTH == FRAME_WIDTH and DISPLAY_HEIGHT == FRAME_HEIGHT:
        return img

    original_size = _image_size(img)
    img_w, img_h = original_size
    if img_w is None or img_h is None:
        print("DISPLAY resize fallback, cannot read size")
        return img

    scale_x = DISPLAY_WIDTH / img_w
    scale_y = DISPLAY_HEIGHT / img_h
    display_img = None

    try:
        if hasattr(img, "scale"):
            try:
                display_img = img.scale(scale_x, scale_y)
            except Exception as error:
                print("DISPLAY scale failed: %s" % error)
                display_img = None

        if display_img is None and hasattr(img, "copy") and hasattr(img, "scale"):
            try:
                display_img = img.copy()
                display_img = display_img.scale(scale_x, scale_y)
            except Exception as error:
                print("DISPLAY copy+scale failed: %s" % error)
                display_img = None
    except Exception as error:
        print("DISPLAY resize failed: %s" % error)

    if display_img is None:
        print("DISPLAY resize fallback, original=%s" % (original_size,))
        return img

    new_size = _image_size(display_img)
    print("DISPLAY resized %s -> %s" % (original_size, new_size))
    return display_img


def draw_detections(img, detections, img_w=FRAME_WIDTH, img_h=FRAME_HEIGHT):
    for det in detections:
        if det.class_id >= len(CLASS_NAMES):
            continue

        name = CLASS_NAMES[det.class_id]
        color = DRAW_COLORS.get(name, (255, 255, 255))
        x = int((det.cx - det.w / 2) * img_w / 1000)
        y = int((det.cy - det.h / 2) * img_h / 1000)
        w = int(det.w * img_w / 1000)
        h = int(det.h * img_h / 1000)

        if hasattr(img, "draw_rectangle"):
            img.draw_rectangle((x, y, w, h), color=color, thickness=2)
        draw_label(img, x, max(0, y - 18), "%s %d" % (name, det.score), color)


def draw_label(img, x, y, text, color):
    if hasattr(img, "draw_string_advanced"):
        try:
            img.draw_string_advanced(x, y, 16, text, color)
            return
        except TypeError:
            try:
                img.draw_string_advanced(x, y, text, color=color, scale=1)
                return
            except TypeError:
                pass

    if hasattr(img, "draw_string"):
        img.draw_string(x, y, text, color=color)


def main():
    sensor = None
    frame_id = 0
    last_send = 0

    try:
        uart = setup_uart()
        model_path = resolve_model_path()
        print("Loading model: %s" % model_path)
        detector = K230YoloDetector(model_path)

        sensor = Sensor(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(sensor_pixformat_value(Sensor))

        if SHOW_IMAGE:
            if DISPLAY_TO_IDE:
                Display.init(
                    Display.VIRT,
                    width=DISPLAY_WIDTH,
                    height=DISPLAY_HEIGHT,
                    to_ide=True,
                )
            else:
                Display.init(Display.VIRT, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, fps=30)

        MediaManager.init()
        sensor.run()

        while True:
            os.exitpoint()
            img = sensor.snapshot()
            detections = detector.detect(img)

            if SHOW_IMAGE:
                display_img = resize_for_display(img)
                draw_detections(display_img, detections, DISPLAY_WIDTH, DISPLAY_HEIGHT)
                try:
                    Display.show_image(display_img)
                except Exception as error:
                    print("DISPLAY show_image failed: %s, size=%s" % (error, _image_size(display_img)))

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
