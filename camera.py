import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import os
import sys
import socket
import struct
import time
from pathlib import Path

import numpy as np
import cv2
import hailo

from hailo_apps.hailo_app_python.core.common.buffer_utils import (
    get_caps_from_pad,
    get_numpy_from_buffer,
)
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.pose_estimation.pose_estimation_pipeline import (
    GStreamerPoseEstimationApp,
)

# -----------------------------------------------------------------------------------------------
# SIYI Gimbal Control Class
# -----------------------------------------------------------------------------------------------

class SiyiGimbal:
    def __init__(self, ip="192.168.144.25", port=37260):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def _crc16_cal(self, data, crc_init=0):
        crc = crc_init
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return crc

    def send_packet(self, cmd_id, data_bytes: bytes):
        """Create and send SIYI protocol packet over UDP."""
        STX = 0x5566
        CTRL = 0x01  # Usually 0x01 for data that may require ACK

        data_len = len(data_bytes)

        # Body: CTRL(1) + DataLen(2) + Seq(2) + CmdID(1) + Data(N)
        body_prefix = struct.pack("<BHHB", CTRL, data_len, self.seq, cmd_id)
        body = body_prefix + data_bytes

        # Header: STX(2)
        header = struct.pack("<H", STX)
        full_msg_no_crc = header + body

        # CRC over full message (without CRC itself)
        crc_val = self._crc16_cal(full_msg_no_crc)
        final_packet = full_msg_no_crc + struct.pack("<H", crc_val)

        self.sock.sendto(final_packet, (self.ip, self.port))
        self.seq = (self.seq + 1) & 0xFFFF

    def set_lock_mode(self):
        """Set gimbal to Lock Mode (SDK: CMD_ID=0x0C, Data=0x03)."""
        print("[GIMBAL] Setting LOCK mode...")
        CMD_ID = 0x0C
        data = struct.pack("<B", 0x03)  # 0x03 = Lock mode (per SIYI SDK)
        self.send_packet(CMD_ID, data)

    def send_speed(self, yaw_speed: int, pitch_speed: int):
        """Send speed command to gimbal (-100 ~ 100)."""
        CMD_ID = 0x07

        yaw_speed = int(max(-100, min(100, yaw_speed)))
        pitch_speed = int(max(-100, min(100, pitch_speed)))

        # Data: yaw (int8), pitch (int8)
        data = struct.pack("<bb", yaw_speed, pitch_speed)
        self.send_packet(CMD_ID, data)

        print(f"[GIMBAL] send_speed yaw={yaw_speed}, pitch={pitch_speed}")


# -----------------------------------------------------------------------------------------------
# Tracking Logic Class
# -----------------------------------------------------------------------------------------------

class TrackingCallback(app_callback_class):
    def __init__(self):
        super().__init__()

        # Gimbal controller
        self.gimbal = SiyiGimbal()
        self.gimbal.set_lock_mode()

        # Rate limiting for commands
        self.last_command_time = time.time()
        self.command_interval = 0.10  # seconds (10 Hz)

        # Simple P-gain control
        self.kp_x = 60.0  # yaw gain
        self.kp_y = 60.0  # pitch gain

        # Deadzone around image center to avoid jitter
        self.deadzone = 0.08  # 8%

        # For frame access (if app supports it)
        self.use_frame = True


# -----------------------------------------------------------------------------------------------
# Callback Function
# -----------------------------------------------------------------------------------------------

def app_callback(pad, info, user_data: TrackingCallback):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()

    # Get ROI and all detection objects
    roi = hailo.get_roi_from_buffer(buffer)

    # Try to fetch detection objects
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    if user_data.get_count() % 30 == 0:
        print(f"[DEBUG] Frame #{user_data.get_count()} | detections={len(detections)}")

    # Select best person detection
    target_bbox = None
    max_conf = 0.0

    for det in detections:
        label = det.get_label()
        conf = det.get_confidence()

        if label == "person" and conf > 0.6:
            if conf > max_conf:
                max_conf = conf
                target_bbox = det.get_bbox()

    # Get frame size from caps (needed for pixel -> normalized conversion)
    fmt, width, height = get_caps_from_pad(pad)

    # For visualization (if supported by app)
    frame = None
    if user_data.use_frame and fmt is not None and width is not None and height is not None:
        frame = get_numpy_from_buffer(buffer, fmt, width, height)
        # Hailo usually returns RGB, convert to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Tracking and gimbal command
    now = time.time()
    if now - user_data.last_command_time >= user_data.command_interval:
        yaw_cmd = 0
        pitch_cmd = 0

        if target_bbox is not None:
            # Raw bbox values
            x = target_bbox.xmin()
            y = target_bbox.ymin()
            w = target_bbox.width()
            h = target_bbox.height()

            # Debug: print raw bbox occasionally
            print(f"[BBOX] raw xmin={x:.3f}, ymin={y:.3f}, w={w:.3f}, h={h:.3f}")

            # Decide if bbox is normalized or in pixels
            # If greater than 1.0, treat as pixels and normalize by width/height
            if width is not None and height is not None and (
                x > 1.0 or y > 1.0 or w > 1.0 or h > 1.0
            ):
                cx = (x + w / 2.0) / float(width)
                cy = (y + h / 2.0) / float(height)
            else:
                cx = x + w / 2.0
                cy = y + h / 2.0

            # Error relative to center (0.5, 0.5)
            err_x = cx - 0.5
            err_y = cy - 0.5

            # Deadzone
            if abs(err_x) < user_data.deadzone:
                err_x = 0.0
            if abs(err_y) < user_data.deadzone:
                err_y = 0.0

            # P control for yaw/pitch
            yaw_cmd = int(err_x * user_data.kp_x)
            pitch_cmd = int(err_y * user_data.kp_y)

            # Invert pitch (usually camera needs inverse direction)
            pitch_cmd = -pitch_cmd

            print(
                f"[TRACK] cx={cx:.3f}, cy={cy:.3f}, "
                f"err=({err_x:.3f}, {err_y:.3f}) -> cmd=({yaw_cmd}, {pitch_cmd}), conf={max_conf:.2f}"
            )

            # Draw bbox and center on frame (for debugging overlay)
            if frame is not None:
                # Convert normalized back to pixel for drawing if needed
                if cx <= 1.5 and cy <= 1.5:  # assume normalized
                    bx = int(cx * width)
                    by = int(cy * height)
                else:
                    bx = int(cx)
                    by = int(cy)

                cv2.circle(frame, (bx, by), 6, (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"person {max_conf:.2f}",
                    (bx + 10, by),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
        else:
            print("[TRACK] no person detected -> cmd=(0, 0)")

        # Send command to gimbal
        user_data.gimbal.send_speed(yaw_cmd, pitch_cmd)
        user_data.last_command_time = now

    # Optional: store frame in user_data (depending on app implementation)
    if frame is not None:
        user_data.set_frame(frame)

    return Gst.PadProbeReturn.OK


# -----------------------------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------------------------

if __name__ == "__main__":
    # Initialize GStreamer
    Gst.init(None)

    # Set Hailo env file
    project_root = Path(__file__).resolve().parent.parent
    env_file = project_root / ".env"
    os.environ["HAILO_ENV_FILE"] = str(env_file)

    # Auto-add input (/dev/video0) if not provided
    if "--input" not in sys.argv:
        sys.argv.append("--input")
        sys.argv.append("/dev/video0")

    # Create callback user data
    user_data = TrackingCallback()

    # Create and run app
    app = GStreamerPoseEstimationApp(app_callback, user_data)
    app.run()
