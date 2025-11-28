import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import sys
import numpy as np
import cv2
import hailo
import socket
import struct
import time
from pathlib import Path

from hailo_apps.hailo_app_python.core.common.buffer_utils import (
    get_caps_from_pad, get_numpy_from_buffer
)
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.pose_estimation.pose_estimation_pipeline import (
    GStreamerPoseEstimationApp
)

# -----------------------------------------------------------------------------------
# SIYI Gimbal Control Class (FIXED PACKET FORMAT)
# -----------------------------------------------------------------------------------
class SiyiGimbal:
    def __init__(self, ip="192.168.144.25", port=37260, debug=True):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.debug = debug

    @staticmethod
    def _crc16_cal(data, crc_init=0x0000):
        """Modbus-style CRC16 (same as many SIYI examples)"""
        crc = crc_init
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def send_packet(self, cmd_id, data_bytes: bytes):
        """
        SIYI packet format (typical):
        STX(2)  = 0x55 0x66
        CTRL(1) = 0x01
        LEN(2)  = data length (N)
        SEQ(2)  = sequence
        CMD(1)  = command id
        DATA(N)
        CRC(2)  = CRC16 over [CTRL..DATA]
        """
        STX = b'\x55\x66'          # FIXED: raw bytes, not 16-bit little-endian
        CTRL = 0x01

        data_len = len(data_bytes)
        seq = self.seq & 0xFFFF

        body = struct.pack('<BHHB', CTRL, data_len, seq, cmd_id) + data_bytes

        # FIXED: CRC over body only (CTRL..DATA), not including STX
        crc_val = self._crc16_cal(body)
        crc_bytes = struct.pack('<H', crc_val)

        packet = STX + body + crc_bytes

        if self.debug:
            print(f"[GIMBAL] send_packet cmd=0x{cmd_id:02X}, len={data_len}, seq={seq}")
            print("[GIMBAL] bytes:", packet.hex(" "))

        self.sock.sendto(packet, (self.ip, self.port))
        self.seq += 1

    def set_lock_mode(self):
        """Lock mode (you can change 0x03 -> other mode if needed)"""
        print("[GIMBAL] set_lock_mode()")
        # Many docs: CMD 0x0C, data: 0x03 (Lock)
        self.send_packet(0x0C, struct.pack('<B', 0x03))

    def send_speed(self, yaw_speed: int, pitch_speed: int):
        """
        Speed command
        - CMD 0x07
        - Data: signed int8 yaw, signed int8 pitch  (-100~100 recommended)
        """
        yaw_speed = max(-100, min(100, int(yaw_speed)))
        pitch_speed = max(-100, min(100, int(pitch_speed)))

        if self.debug:
            print(f"[GIMBAL] send_speed yaw={yaw_speed}, pitch={pitch_speed}")

        self.send_packet(0x07, struct.pack('<bb', yaw_speed, pitch_speed))

# -----------------------------------------------------------------------------------
# Tracking Callback
# -----------------------------------------------------------------------------------
class TrackingCallback(app_callback_class):
    def __init__(self):
        super().__init__()
        self.gimbal = SiyiGimbal()
        self.gimbal.set_lock_mode()

        self.last_command_time = time.time()
        self.command_interval = 0.10  # 10 Hz

        # PID gains
        self.kp_x = 80.0   # yaw gain
        self.kp_y = 80.0   # pitch gain
        self.deadzone = 0.08

    def process_detections(self, detections):
        target_bbox = None
        max_conf = 0.0

        for det in detections:
            label = det.get_label()
            conf = det.get_confidence()

            if label == "person" and conf > 0.5:
                if conf > max_conf:
                    max_conf = conf
                    target_bbox = det.get_bbox()

        return target_bbox, max_conf

# -----------------------------------------------------------------------------------
# GStreamer callback
# -----------------------------------------------------------------------------------
def app_callback(pad, info, user_data: TrackingCallback):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()
    frame_idx = user_data.get_count()

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    target_bbox, conf = user_data.process_detections(detections)

    now = time.time()
    if now - user_data.last_command_time >= user_data.command_interval:
        yaw_cmd = 0
        pitch_cmd = 0

        if target_bbox is not None:
            cx = target_bbox.xmin() + target_bbox.width() / 2.0
            cy = target_bbox.ymin() + target_bbox.height() / 2.0

            err_x = cx - 0.5
            err_y = cy - 0.5

            if abs(err_x) < user_data.deadzone:
                err_x = 0.0
            if abs(err_y) < user_data.deadzone:
                err_y = 0.0

            yaw_cmd = int(err_x * user_data.kp_x)
            pitch_cmd = int(-err_y * user_data.kp_y)  # invert pitch

            print(
                f"[TRACK] frame={frame_idx}, "
                f"cx={cx:.3f}, cy={cy:.3f}, "
                f"err=({err_x:.3f},{err_y:.3f}) -> cmd=({yaw_cmd},{pitch_cmd}), conf={conf:.2f}"
            )
        else:
            print(f"[TRACK] frame={frame_idx}, no person detected -> cmd=(0,0)")

        user_data.gimbal.send_speed(yaw_cmd, pitch_cmd)
        user_data.last_command_time = now

    return Gst.PadProbeReturn.OK

# -----------------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------------
if __name__ == "__main__":
    # Hailo env
    project_root = Path(__file__).resolve().parent.parent
    env_file = project_root / ".env"
    os.environ["HAILO_ENV_FILE"] = str(env_file)

    # If no --input in args, use /dev/video0 (USB cam)
    if "--input" not in sys.argv:
        sys.argv += ["--input", "/dev/video0"]

    user_data = TrackingCallback()
    app = GStreamerPoseEstimationApp(app_callback, user_data)
    app.run()