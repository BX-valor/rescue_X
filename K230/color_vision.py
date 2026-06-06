"""Four-color ball and purple safe-zone detection for CanMV K230."""

import gc
import os
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor

from color_config import (
    BALL_MAX_ASPECT,
    BALL_MAX_BOX_AREA,
    BALL_MAX_FILL,
    BALL_MAX_PIXELS,
    BALL_MIN_ASPECT,
    BALL_MIN_BOX_AREA,
    BALL_MIN_FILL,
    BALL_MIN_PIXELS,
    BALL_THRESHOLDS,
    FRAME_HEIGHT,
    FRAME_WIDTH,
    PRINT_INTERVAL_MS,
    SAFE_MAX_ASPECT,
    SAFE_MAX_FILL,
    SAFE_MIN_ASPECT,
    SAFE_MIN_HEIGHT,
    SAFE_MIN_PIXELS,
    SAFE_MIN_WIDTH,
    SAFE_ZONE_THRESHOLD,
    SHOW_IMAGE,
    TRACK_CONFIRM_FRAMES,
    TRACK_MATCH_DISTANCE,
    TRACK_MAX_MISSES,
)


DRAW_COLORS = {
    "red": (255, 0, 0),
    "blue": (0, 80, 255),
    "yellow": (255, 220, 0),
    "black": (255, 255, 255),
    "purple": (200, 0, 255),
}


class BallDetection:
    def __init__(self, color, x, y, width, height, pixels):
        self.color = color
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.cx = x + width // 2
        self.cy = y + height // 2
        self.pixels = pixels
        self.stable_frames = 1
        self.in_safe_zone = False

    def distance_sq(self, other):
        dx = self.cx - other.cx
        dy = self.cy - other.cy
        return dx * dx + dy * dy


class SafeZoneDetection:
    def __init__(self, x, y, width, height, pixels):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.cx = x + width // 2
        self.cy = y + height // 2
        self.pixels = pixels

    def contains(self, x, y, margin=0):
        return (
            self.x + margin <= x <= self.x + self.width - margin
            and self.y + margin <= y <= self.y + self.height - margin
        )


class BallTrack:
    def __init__(self, detection):
        self.detection = detection
        self.hits = 1
        self.misses = 0

    def update(self, detection):
        self.detection = detection
        self.hits += 1
        self.misses = 0
        self.detection.stable_frames = self.hits


class DetectionTracker:
    def __init__(self):
        self.tracks = []

    def update(self, detections):
        matched_track_ids = set()

        for detection in detections:
            best_track = None
            best_distance = None

            for track in self.tracks:
                if id(track) in matched_track_ids:
                    continue
                if track.detection.color != detection.color:
                    continue

                distance = track.detection.distance_sq(detection)
                dynamic_limit = max(
                    TRACK_MATCH_DISTANCE,
                    detection.width,
                    detection.height,
                )
                limit_sq = dynamic_limit * dynamic_limit
                if distance <= limit_sq and (
                    best_distance is None or distance <= best_distance
                ):
                    best_distance = distance
                    best_track = track

            if best_track:
                best_track.update(detection)
                matched_track_ids.add(id(best_track))
            else:
                track = BallTrack(detection)
                self.tracks.append(track)
                matched_track_ids.add(id(track))

        alive_tracks = []
        for track in self.tracks:
            if id(track) not in matched_track_ids:
                track.misses += 1
            if track.misses <= TRACK_MAX_MISSES:
                alive_tracks.append(track)
        self.tracks = alive_tracks

        return [
            track.detection
            for track in self.tracks
            if track.hits >= TRACK_CONFIRM_FRAMES and track.misses == 0
        ]


class ColorVision:
    def __init__(self):
        self.tracker = DetectionTracker()

    @staticmethod
    def _blob_values(blob):
        return (
            blob.x(),
            blob.y(),
            blob.w(),
            blob.h(),
            blob.pixels(),
        )

    @staticmethod
    def _valid_ball(width, height, pixels):
        if width <= 0 or height <= 0:
            return False

        box_area = width * height
        aspect = width / height
        fill = pixels / box_area

        return (
            BALL_MIN_PIXELS <= pixels <= BALL_MAX_PIXELS
            and BALL_MIN_BOX_AREA <= box_area <= BALL_MAX_BOX_AREA
            and BALL_MIN_ASPECT <= aspect <= BALL_MAX_ASPECT
            and BALL_MIN_FILL <= fill <= BALL_MAX_FILL
        )

    def detect_balls(self, img):
        detections = []

        for color, threshold in BALL_THRESHOLDS.items():
            blobs = img.find_blobs(
                [threshold],
                pixels_threshold=BALL_MIN_PIXELS,
                area_threshold=BALL_MIN_BOX_AREA,
                merge=True,
                margin=3,
            )

            for blob in blobs:
                x, y, width, height, pixels = self._blob_values(blob)
                if self._valid_ball(width, height, pixels):
                    detections.append(
                        BallDetection(color, x, y, width, height, pixels)
                    )

        return detections

    @staticmethod
    def _valid_safe_zone(width, height, pixels):
        if width <= 0 or height <= 0:
            return False

        box_area = width * height
        aspect = width / height
        fill = pixels / box_area

        return (
            width >= SAFE_MIN_WIDTH
            and height >= SAFE_MIN_HEIGHT
            and pixels >= SAFE_MIN_PIXELS
            and SAFE_MIN_ASPECT <= aspect <= SAFE_MAX_ASPECT
            and fill <= SAFE_MAX_FILL
        )

    def detect_safe_zone(self, img):
        blobs = img.find_blobs(
            [SAFE_ZONE_THRESHOLD],
            pixels_threshold=30,
            area_threshold=40,
            merge=True,
            margin=12,
        )

        valid = []
        raw = []
        for blob in blobs:
            values = self._blob_values(blob)
            raw.append(values)
            if self._valid_safe_zone(values[2], values[3], values[4]):
                valid.append(values)

        if valid:
            best = max(valid, key=lambda item: item[2] * item[3])
            return SafeZoneDetection(*best)

        # Reflections can split a hollow frame into several purple strips.
        if len(raw) < 2:
            return None

        x1 = min(item[0] for item in raw)
        y1 = min(item[1] for item in raw)
        x2 = max(item[0] + item[2] for item in raw)
        y2 = max(item[1] + item[3] for item in raw)
        pixels = sum(item[4] for item in raw)
        width = x2 - x1
        height = y2 - y1

        if self._valid_safe_zone(width, height, pixels):
            return SafeZoneDetection(x1, y1, width, height, pixels)
        return None

    def process(self, img):
        safe_zone = self.detect_safe_zone(img)
        balls = self.tracker.update(self.detect_balls(img))

        if safe_zone:
            for ball in balls:
                ball.in_safe_zone = safe_zone.contains(ball.cx, ball.cy, margin=3)

        return balls, safe_zone


def draw_results(img, balls, safe_zone):
    if safe_zone:
        img.draw_rectangle(
            (safe_zone.x, safe_zone.y, safe_zone.width, safe_zone.height),
            color=DRAW_COLORS["purple"],
            thickness=3,
        )
        img.draw_string(
            safe_zone.x,
            max(0, safe_zone.y - 14),
            "SAFE",
            color=DRAW_COLORS["purple"],
        )

    for ball in balls:
        color = DRAW_COLORS[ball.color]
        img.draw_rectangle(
            (ball.x, ball.y, ball.width, ball.height),
            color=color,
            thickness=2,
        )
        img.draw_cross(ball.cx, ball.cy, color=color, size=8, thickness=2)
        suffix = " IN" if ball.in_safe_zone else ""
        img.draw_string(
            ball.x,
            max(0, ball.y - 12),
            "%s %d%s" % (ball.color, ball.stable_frames, suffix),
            color=color,
        )


def print_results(balls, safe_zone):
    if safe_zone:
        print(
            "ZONE,purple,%d,%d,%d,%d"
            % (
                safe_zone.cx,
                safe_zone.cy,
                safe_zone.width,
                safe_zone.height,
            )
        )
    else:
        print("ZONE,none")

    if not balls:
        print("BALL,none")
        return

    for ball in balls:
        print(
            "BALL,%s,%d,%d,%d,%d,%d"
            % (
                ball.color,
                ball.cx,
                ball.cy,
                ball.width,
                ball.height,
                1 if ball.in_safe_zone else 0,
            )
        )


def main():
    sensor = None
    last_print = 0

    try:
        sensor = Sensor(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)

        if SHOW_IMAGE:
            Display.init(
                Display.VIRT,
                width=FRAME_WIDTH,
                height=FRAME_HEIGHT,
                fps=30,
            )

        MediaManager.init()
        sensor.run()
        vision = ColorVision()

        while True:
            os.exitpoint()
            img = sensor.snapshot()
            balls, safe_zone = vision.process(img)
            draw_results(img, balls, safe_zone)

            if SHOW_IMAGE:
                Display.show_image(img)

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= PRINT_INTERVAL_MS:
                print_results(balls, safe_zone)
                last_print = now

            gc.collect()

    except KeyboardInterrupt:
        print("Stopped by user")
    except BaseException as error:
        print("Vision error: %s" % error)
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
