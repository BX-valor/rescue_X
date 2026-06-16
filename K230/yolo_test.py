"""Standalone YOLO test for K230.

Run this before connecting ESP32. It loads the same model as yolo_sender.py,
draws detection boxes on the K230 display, and prints detections to the IDE
serial terminal.
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

from yolo_sender import (
    CLASS_NAMES,
    DISPLAY_HEIGHT,
    DISPLAY_WIDTH,
    FRAME_HEIGHT,
    FRAME_WIDTH,
    MODEL_INPUT_LAYOUT,
    SENSOR_PIXFORMAT,
    K230YoloDetector,
    _image_size,
    draw_detections,
    resize_for_display,
    resolve_model_path,
    sensor_pixformat_value,
    ticks_diff,
    ticks_ms,
)


PRINT_INTERVAL_MS = 500
ENABLE_DISPLAY = False
DISPLAY_TO_IDE = True
DEBUG_FRAME_STEPS = False
DEBUG_SINGLE_FRAME = False
SAVE_DEBUG_IMAGES = False
SAVE_INTERVAL_MS = 1000
SAVE_MAX_IMAGES = 30
SAVE_DIR = "/sdcard/rescue_x/debug_frames"


def print_detections(detections):
    if not detections:
        print("YOLO none")
        return

    for det in detections:
        if det.class_id < len(CLASS_NAMES):
            name = CLASS_NAMES[det.class_id]
        else:
            name = "class_%d" % det.class_id

        print(
            "YOLO,%s,%d,%d,%d,%d,%d"
            % (name, det.score, det.cx, det.cy, det.w, det.h)
        )


def ensure_dir(path):
    try:
        os.mkdir(path)
    except OSError:
        pass


def save_debug_image(img, index):
    if not hasattr(img, "save"):
        return False

    ensure_dir(SAVE_DIR)
    path = "%s/frame_%03d.jpg" % (SAVE_DIR, index)
    try:
        save_img = img
        if hasattr(img, "to_rgb565"):
            try:
                save_img = img.to_rgb565(copy=True)
            except TypeError:
                save_img = img.to_rgb565()
        save_img.save(path)
        print("SAVED,%s" % path)
        return True
    except Exception as error:
        print("SAVE_ERROR,%s,%s" % (path, error))
        return False


def main():
    sensor = None
    last_print = 0
    last_save = 0
    saved_count = 0

    try:
        model_path = resolve_model_path()
        print("Loading model: %s" % model_path)
        print(
            "Input layout: %s, pixformat: %s, size: %dx%d"
            % (MODEL_INPUT_LAYOUT, SENSOR_PIXFORMAT, FRAME_WIDTH, FRAME_HEIGHT)
        )
        detector = K230YoloDetector(model_path)

        print("INIT sensor")
        sensor = Sensor(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(sensor_pixformat_value(Sensor))
        print("INIT sensor OK")

        if ENABLE_DISPLAY:
            print("INIT display")
            if DISPLAY_TO_IDE:
                Display.init(
                    Display.VIRT,
                    width=DISPLAY_WIDTH,
                    height=DISPLAY_HEIGHT,
                    to_ide=True,
                )
            else:
                Display.init(
                    Display.VIRT,
                    width=DISPLAY_WIDTH,
                    height=DISPLAY_HEIGHT,
                    fps=30,
                )
            print("INIT display OK")

        print("INIT media")
        MediaManager.init()
        print("INIT media OK")
        print("START sensor")
        sensor.run()
        print("START sensor OK")

        print("YOLO test started")
        while True:
            os.exitpoint()
            if DEBUG_FRAME_STEPS:
                print("FRAME snapshot")
            img = sensor.snapshot()
            if DEBUG_FRAME_STEPS:
                print("FRAME snapshot OK")
                print("FRAME detect")
            detections = detector.detect(img)
            if DEBUG_FRAME_STEPS:
                print("FRAME detect OK")
            if ENABLE_DISPLAY:
                if DEBUG_FRAME_STEPS:
                    print("FRAME draw")
                display_img = resize_for_display(img)
                draw_detections(display_img, detections, DISPLAY_WIDTH, DISPLAY_HEIGHT)
                if DEBUG_FRAME_STEPS:
                    print("FRAME show")
                disp_w, disp_h = _image_size(display_img)
                if disp_w is None or disp_w > DISPLAY_WIDTH or disp_h > DISPLAY_HEIGHT:
                    print("DISPLAY skip, resize failed, size=%dx%d" %
                          (disp_w or 0, disp_h or 0))
                else:
                    Display.show_image(display_img)
                if DEBUG_FRAME_STEPS:
                    print("FRAME show OK")

            now = ticks_ms()
            if ticks_diff(now, last_print) >= PRINT_INTERVAL_MS:
                print_detections(detections)
                last_print = now

            if (
                SAVE_DEBUG_IMAGES
                and saved_count < SAVE_MAX_IMAGES
                and ticks_diff(now, last_save) >= SAVE_INTERVAL_MS
            ):
                if save_debug_image(img, saved_count):
                    saved_count += 1
                last_save = now

            gc.collect()

            if DEBUG_SINGLE_FRAME:
                print("DEBUG single frame done")
                break

    except KeyboardInterrupt:
        print("Stopped by user")
    except BaseException as error:
        print("YOLO test error: %s" % error)
        raise
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        if ENABLE_DISPLAY:
            Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
