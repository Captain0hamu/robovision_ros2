#!/usr/bin/env python3
"""Publish RGB, monocular depth, CameraInfo and a colored point cloud from video."""

import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
import torch.nn.functional as torch_functional
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from transformers import AutoImageProcessor, AutoModelForDepthEstimation


class MonocularDepthPublisher(Node):
    def __init__(self):
        super().__init__("monocular_depth_publisher")
        self.declare_parameter("video_path", "")
        self.declare_parameter("model", "LiheYoung/depth-anything-small-hf")
        self.declare_parameter("loop", True)
        self.declare_parameter("near_m", 0.5)
        self.declare_parameter("far_m", 5.0)
        self.declare_parameter("horizontal_fov_deg", 70.0)
        self.declare_parameter("point_step", 6)
        self.declare_parameter("frame_stride", 1)
        self.declare_parameter("use_gpu", True)

        video_path = Path(self.get_parameter("video_path").value).expanduser()
        if not video_path.is_file():
            raise RuntimeError(f"video_path does not exist: {video_path}")

        self.capture = cv2.VideoCapture(str(video_path))
        if not self.capture.isOpened():
            raise RuntimeError(f"cannot open video: {video_path}")
        self.fps = self.capture.get(cv2.CAP_PROP_FPS) or 30.0
        self.loop = self.get_parameter("loop").value
        self.near_m = float(self.get_parameter("near_m").value)
        self.far_m = float(self.get_parameter("far_m").value)
        self.horizontal_fov = math.radians(
            float(self.get_parameter("horizontal_fov_deg").value)
        )
        self.point_step = max(1, int(self.get_parameter("point_step").value))
        self.frame_stride = max(1, int(self.get_parameter("frame_stride").value))
        self.frame_id = "camera_optical_frame"
        self.bridge = CvBridge()

        use_gpu = self.get_parameter("use_gpu").value and torch.cuda.is_available()
        self.device = torch.device("cuda" if use_gpu else "cpu")
        model_name = self.get_parameter("model").value
        self.get_logger().info(f"Loading {model_name} on {self.device} ...")
        self.processor = AutoImageProcessor.from_pretrained(model_name)
        self.model = AutoModelForDepthEstimation.from_pretrained(model_name)
        self.model.to(self.device).eval()

        self.rgb_pub = self.create_publisher(
            Image, "/camera/rgb/image_raw", qos_profile_sensor_data
        )
        self.depth_pub = self.create_publisher(
            Image, "/camera/depth/image_raw", qos_profile_sensor_data
        )
        self.info_pub = self.create_publisher(
            CameraInfo, "/camera/rgb/camera_info", qos_profile_sensor_data
        )
        self.points_pub = self.create_publisher(
            PointCloud2, "/camera/depth/points", qos_profile_sensor_data
        )
        # Inference determines the effective rate. The timer only schedules the next frame.
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info(f"Reading {video_path} at up to {self.fps:g} fps")

    def read_frame(self):
        ok, frame = self.capture.read()
        if not ok and self.loop:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.capture.read()
        if not ok:
            return None
        for _ in range(self.frame_stride - 1):
            if not self.capture.grab():
                break
        return frame

    def estimate_depth(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb, return_tensors="pt")
        inputs = {key: value.to(self.device) for key, value in inputs.items()}
        with torch.inference_mode():
            prediction = self.model(**inputs).predicted_depth
            prediction = torch_functional.interpolate(
                prediction.unsqueeze(1),
                size=bgr.shape[:2],
                mode="bicubic",
                align_corners=False,
            )[0, 0]
        relative = prediction.float().cpu().numpy()

        # Depth Anything produces relative inverse depth (larger means closer).
        low, high = np.percentile(relative, (2.0, 98.0))
        normalized = np.clip((relative - low) / max(high - low, 1e-6), 0.0, 1.0)
        return (self.far_m - normalized * (self.far_m - self.near_m)).astype(
            np.float32
        )

    def camera_info(self, width, height, stamp):
        fx = width / (2.0 * math.tan(self.horizontal_fov / 2.0))
        fy = fx
        cx = (width - 1.0) / 2.0
        cy = (height - 1.0) / 2.0
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def point_cloud(self, bgr, depth, info, stamp):
        rows = np.arange(0, depth.shape[0], self.point_step)
        cols = np.arange(0, depth.shape[1], self.point_step)
        u, v = np.meshgrid(cols, rows)
        z = depth[v, u]
        x = (u.astype(np.float32) - info.k[2]) * z / info.k[0]
        y = (v.astype(np.float32) - info.k[5]) * z / info.k[4]
        colors = bgr[v, u]
        packed_rgb = (
            (colors[..., 2].astype(np.uint32) << 16)
            | (colors[..., 1].astype(np.uint32) << 8)
            | colors[..., 0].astype(np.uint32)
        )

        points = np.empty(z.size, dtype=[
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")
        ])
        points["x"] = x.ravel()
        points["y"] = y.ravel()
        points["z"] = z.ravel()
        points["rgb"] = packed_rgb.ravel()

        cloud = PointCloud2()
        cloud.header = Header(stamp=stamp, frame_id=self.frame_id)
        cloud.height = 1
        cloud.width = points.size
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = points.dtype.itemsize
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = points.tobytes()
        return cloud

    def publish_frame(self):
        frame = self.read_frame()
        if frame is None:
            self.get_logger().info("End of video")
            self.timer.cancel()
            return

        depth = self.estimate_depth(frame)
        stamp = self.get_clock().now().to_msg()
        height, width = frame.shape[:2]
        info = self.camera_info(width, height, stamp)

        rgb_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self.frame_id
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.frame_id

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info)
        self.points_pub.publish(self.point_cloud(frame, depth, info, stamp))

    def destroy_node(self):
        self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MonocularDepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGTERM can invalidate the ROS context while an inference callback is publishing.
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        if node.context.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
