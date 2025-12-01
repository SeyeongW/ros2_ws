#!/usr/bin/env python3
import sys
import threading
import time
import os

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# GStreamer Imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

class SiyiHailoPoseNode(Node):
    """
    SIYI A8 Mini + Hailo-8 (Docker) Pose Estimation Node
    """

    def __init__(self):
        super().__init__('siyi_hailo_pose_node')

        # --- [1] 사용자 설정 (여기만 확인하세요) ---
        # 내 노트북(PC) IP 주소 (여기로 영상이 전송됩니다)
        self.declare_parameter('pc_ip', '192.168.1.100') 
        self.declare_parameter('pc_port', 5000)
        
        # SIYI 카메라 주소 (기본값)
        self.declare_parameter('rtsp_url', 'rtsp://192.168.144.25:8554/main.264')

        self.declare_parameter('hef_path', '/hailo_ws/resources/models/hailo8/yolov8m_pose.hef')
        
        # 후처리 파일: Pose 전용 라이브러리 사용
        self.declare_parameter('post_process_so', '/hailo_ws/resources/so/libyolov8pose_postprocess.so')
        
        self.declare_parameter('stream_bitrate', 2000)

        # 변수 로드
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.pc_ip = self.get_parameter('pc_ip').value
        self.pc_port = self.get_parameter('pc_port').value
        self.hef_path = self.get_parameter('hef_path').value
        self.pp_so = self.get_parameter('post_process_so').value
        self.bitrate = self.get_parameter('stream_bitrate').value

        self.get_logger().info(f"--- SIYI Hailo-8 Node Started ---")
        self.get_logger().info(f"Model: {self.hef_path}")
        self.get_logger().info(f"Post-Process: {self.pp_so}")
        self.get_logger().info(f"Streaming to: {self.pc_ip}:{self.pc_port}")

        # --- ROS Publisher ---
        self.publisher_ = self.create_publisher(Image, '/siyi/pose_image', 10)
        self.bridge = CvBridge()

        # --- GStreamer Init ---
        Gst.init(None)
        self.pipeline = self.create_pipeline()
        
        self.loop = GLib.MainLoop()
        self.gst_thread = threading.Thread(target=self.loop.run)
        self.gst_thread.start()

        self.pipeline.set_state(Gst.State.PLAYING)

    def create_pipeline(self):
        # 1. Source & Decode
        source = (
            f"rtspsrc location={self.rtsp_url} latency=200 protocols=tcp ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "queue max-size-buffers=5 ! "
            "videoconvert ! video/x-raw, format=RGB ! "
        )

        # 2. Inference (Hailo-8)
        inference = (
            f"videoscale ! video/x-raw, width=640, height=640 ! "
            f"hailonet hef-path={self.hef_path} ! "
            f"hailofilter so-path={self.pp_so} qos=false ! "
            "hailooverlay ! "
        )

        # 3. Output Split
        tee = "tee name=t "

        # Branch A: UDP Stream to PC
        branch_network = (
            "t. ! queue ! videoconvert ! video/x-raw, format=I420 ! "
            f"x264enc tune=zerolatency bitrate={self.bitrate} speed-preset=superfast ! "
            "rtph264pay ! "
            f"udpsink host={self.pc_ip} port={self.pc_port} sync=false "
        )

        # Branch B: ROS Topic
        branch_ros = (
            "t. ! queue ! videoconvert ! video/x-raw, format=RGB ! "
            "appsink name=ros_sink emit-signals=true max-buffers=1 drop=true "
        )

        pipeline_str = source + inference + tee + branch_network + branch_ros
        return Gst.parse_launch(pipeline_str)

    def on_frame_probe(self, sink):
        sample = sink.emit("pull-sample")
        if not sample: return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success: return Gst.FlowReturn.ERROR

        try:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "siyi_camera_optical_frame"
            msg.height = height
            msg.width = width
            msg.encoding = "rgb8"
            msg.step = width * 3
            msg.data = map_info.data
            self.publisher_.publish(msg)
        except Exception:
            pass
        finally:
            buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def shutdown(self):
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SiyiHailoPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()

if __name__ == '__main__':
    main()