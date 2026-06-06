"""Runtime-tunable settings for Lushan Pi K230 color detection."""

FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# CanMV find_blobs uses LAB thresholds:
# (L min, L max, A min, A max, B min, B max)
#
# These broad defaults are only starting points. Replace them with values
# measured from the real balls and arena using the CanMV threshold editor.
BALL_THRESHOLDS = {
    "red": (15, 80, 20, 80, 5, 70),
    "blue": (10, 75, -10, 35, -80, -10),
    "yellow": (35, 100, -25, 25, 20, 85),
    "black": (0, 35, -20, 20, -20, 20),
}

SAFE_ZONE_THRESHOLD = (10, 80, 15, 75, -70, -5)

# Ball filtering at 320x240. Scale these values with image area if resolution
# changes.
BALL_MIN_PIXELS = 45
BALL_MAX_PIXELS = 9000
BALL_MIN_BOX_AREA = 70
BALL_MAX_BOX_AREA = 12000
BALL_MIN_ASPECT = 0.68
BALL_MAX_ASPECT = 1.47
BALL_MIN_FILL = 0.32
BALL_MAX_FILL = 0.92

# The purple frame is expected to be much larger and less filled than a ball.
SAFE_MIN_WIDTH = 55
SAFE_MIN_HEIGHT = 45
SAFE_MIN_PIXELS = 180
SAFE_MAX_FILL = 0.62
SAFE_MIN_ASPECT = 0.45
SAFE_MAX_ASPECT = 2.20

# A detection must survive several frames before being considered stable.
TRACK_CONFIRM_FRAMES = 3
TRACK_MAX_MISSES = 3
TRACK_MATCH_DISTANCE = 30

SHOW_IMAGE = True
PRINT_INTERVAL_MS = 200

