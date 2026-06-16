import struct


START_BYTES = bytearray([0xAA, 0xBB])
PACKET_DETECTIONS = 0x01
TEAM_RED = 0x01
MAX_DETECTIONS = 12

CLASS_NAMES = [
    "cross_marker",
    "red_ball",
    "yellow_ball",
    "blue_ball",
    "black_ball",
    "red_safe_zone",
    "blue_safe_zone",
    "purple_boundary",
    "blue_start_zone",
    "red_start_zone",
]


def crc16_ccitt(data, crc=0xFFFF):
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def clamp_u16(value, lower=0, upper=1000):
    value = int(value)
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def normalized_box(x, y, w, h, frame_w, frame_h):
    return [
        clamp_u16(x * 1000 / frame_w),
        clamp_u16(y * 1000 / frame_h),
        clamp_u16(w * 1000 / frame_w),
        clamp_u16(h * 1000 / frame_h),
    ]


def normalized_center_box(cx, cy, w, h, frame_w, frame_h):
    return [
        clamp_u16(cx * 1000 / frame_w),
        clamp_u16(cy * 1000 / frame_h),
        clamp_u16(w * 1000 / frame_w),
        clamp_u16(h * 1000 / frame_h),
    ]


def _field(item, name, default=0):
    if isinstance(item, dict):
        return item.get(name, default)
    return getattr(item, name, default)


def pack_detection_frame(frame_id, detections, team_color=TEAM_RED):
    limited = detections[:MAX_DETECTIONS]
    payload = bytearray([team_color, len(limited)])

    for det in limited:
        item = struct.pack(
            "<BBHHHH",
            int(_field(det, "class_id")) & 0xFF,
            clamp_u16(_field(det, "score"), 0, 100),
            clamp_u16(_field(det, "cx")),
            clamp_u16(_field(det, "cy")),
            clamp_u16(_field(det, "w")),
            clamp_u16(_field(det, "h")),
        )
        payload.extend(item)

    header = bytearray([
        PACKET_DETECTIONS,
        frame_id & 0xFF,
        len(payload) & 0xFF,
    ])
    crc = crc16_ccitt(header + payload)
    return START_BYTES + header + payload + struct.pack(">H", crc)
