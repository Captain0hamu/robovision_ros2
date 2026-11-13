#!/usr/bin/env python3
"""Precompute monocular depth and colored point clouds into ROS 2 bags."""

import argparse
import math
import shutil
from contextlib import nullcontext
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
import torch
import torch.nn.functional as torch_functional
import yaml
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField
from std_msgs.msg import Header
from transformers import AutoImageProcessor, AutoModelForDepthEstimation


RGB_TOPIC = "/camera/rgb/image/compressed"
DEPTH_TOPIC = "/camera/depth/image/compressed"
INFO_TOPIC = "/camera/rgb/camera_info"
POINTS_TOPIC = "/camera/depth/points"
FRAME_ID = "camera_optical_frame"


def read_start_time_ns(source: Path, video: Path) -> int:
    metadata_path = source / "metadata.yaml" if source.is_dir() else None
    if metadata_path and metadata_path.exists():
        metadata = yaml.safe_load(metadata_path.read_text())
        return int(
            metadata["rosbag2_bagfile_information"]["starting_time"]
            ["nanoseconds_since_epoch"]
        )
    return video.stat().st_mtime_ns


def resolve_video(source: Path) -> Path:
    if source.is_file():
        return source
    for name in ("video.mp4", "video.webm"):
        candidate = source / name
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"video.mp4 or video.webm not found in {source}")


def make_camera_info(width, height, stamp, horizontal_fov_deg):
    fx = width / (2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0))
    fy = fx
    cx = (width - 1.0) / 2.0
    cy = (height - 1.0) / 2.0
    message = CameraInfo()
    message.header.stamp.sec = stamp // 1_000_000_000
    message.header.stamp.nanosec = stamp % 1_000_000_000
    message.header.frame_id = FRAME_ID
    message.width = width
    message.height = height
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return message


def make_compressed(frame, stamp, image_format, extension, params=None):
    ok, encoded = cv2.imencode(extension, frame, params or [])
    if not ok:
        raise RuntimeError(f"failed to encode {image_format}")
    message = CompressedImage()
    message.header.stamp.sec = stamp // 1_000_000_000
    message.header.stamp.nanosec = stamp % 1_000_000_000
    message.header.frame_id = FRAME_ID
    message.format = image_format
    message.data = encoded.tobytes()
    return message


def make_point_cloud(bgr, depth, info, stamp, point_step):
    rows = np.arange(0, depth.shape[0], point_step)
    cols = np.arange(0, depth.shape[1], point_step)
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
    points = np.empty(
        z.size,
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")],
    )
    points["x"] = x.ravel()
    points["y"] = y.ravel()
    points["z"] = z.ravel()
    points["rgb"] = packed_rgb.ravel()

    message = PointCloud2()
    message.header = Header(frame_id=FRAME_ID)
    message.header.stamp.sec = stamp // 1_000_000_000
    message.header.stamp.nanosec = stamp % 1_000_000_000
    message.height = 1
    message.width = points.size
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = points.dtype.itemsize
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = points.tobytes()
    return message


class DepthEstimator:
    def __init__(self, model_name, use_gpu, near_m, far_m):
        self.device = torch.device(
            "cuda" if use_gpu and torch.cuda.is_available() else "cpu"
        )
        print(f"Loading {model_name} on {self.device}")
        self.processor = AutoImageProcessor.from_pretrained(model_name)
        self.model = AutoModelForDepthEstimation.from_pretrained(model_name)
        self.model.to(self.device).eval()
        self.near_m = near_m
        self.far_m = far_m

    def __call__(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb, return_tensors="pt")
        inputs = {key: value.to(self.device) for key, value in inputs.items()}
        autocast = (
            torch.autocast(device_type="cuda", dtype=torch.float16)
            if self.device.type == "cuda"
            else nullcontext()
        )
        with torch.inference_mode(), autocast:
            prediction = self.model(**inputs).predicted_depth
            prediction = torch_functional.interpolate(
                prediction.unsqueeze(1),
                size=bgr.shape[:2],
                mode="bicubic",
                align_corners=False,
            )[0, 0]
        relative = prediction.float().cpu().numpy()
        low, high = np.percentile(relative, (2.0, 98.0))
        normalized = np.clip((relative - low) / max(high - low, 1e-6), 0.0, 1.0)
        return (self.far_m - normalized * (self.far_m - self.near_m)).astype(
            np.float32
        )


def create_topics(writer):
    for name, type_name in (
        (RGB_TOPIC, "sensor_msgs/msg/CompressedImage"),
        (DEPTH_TOPIC, "sensor_msgs/msg/CompressedImage"),
        (INFO_TOPIC, "sensor_msgs/msg/CameraInfo"),
        (POINTS_TOPIC, "sensor_msgs/msg/PointCloud2"),
    ):
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=name,
                type=type_name,
                serialization_format="cdr",
                offered_qos_profiles="",
            )
        )


def convert(source, output_root, estimator, frame_stride, point_step, fov, quality):
    video = resolve_video(source)
    output_name = source.name if source.is_dir() else source.stem
    output = output_root / output_name
    if output.exists():
        raise FileExistsError(output)
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {video}")
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    start_ns = read_start_time_ns(source, video)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    create_topics(writer)
    source_index = 0
    output_count = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if source_index % frame_stride != 0:
            source_index += 1
            continue
        stamp = start_ns + round(source_index * 1_000_000_000 / fps)
        depth = estimator(frame)
        height, width = frame.shape[:2]
        info = make_camera_info(width, height, stamp, fov)
        # Monocular depth is not millimetre-accurate. Quantizing to 2 cm greatly
        # reduces PNG size without removing meaningful precision.
        depth_mm = np.clip(np.rint(depth * 50.0) * 20.0, 0, 65535).astype(
            np.uint16
        )
        rgb_message = make_compressed(
            frame,
            stamp,
            "bgr8; jpeg compressed bgr8",
            ".jpg",
            [cv2.IMWRITE_JPEG_QUALITY, quality],
        )
        depth_message = make_compressed(
            depth_mm, stamp, "16UC1; png compressed", ".png"
        )
        points_message = make_point_cloud(
            frame, depth, info, stamp, point_step
        )
        for topic, message in (
            (RGB_TOPIC, rgb_message),
            (DEPTH_TOPIC, depth_message),
            (INFO_TOPIC, info),
            (POINTS_TOPIC, points_message),
        ):
            writer.write(topic, serialize_message(message), stamp)
        output_count += 1
        source_index += 1
        if output_count % 50 == 0:
            print(f"{output_name}: {output_count} frames", flush=True)

    capture.release()
    del writer
    database = next(output.glob("*.db3"))
    old_name = database.name
    database.rename(output / "data.db3")
    metadata = output / "metadata.yaml"
    metadata.write_text(metadata.read_text().replace(old_name, "data.db3"))
    shutil.copy2(video, output / f"video{video.suffix.lower()}")
    print(f"{output_name}: complete ({output_count} synchronized frames)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--model", default="LiheYoung/depth-anything-small-hf")
    parser.add_argument("--frame-stride", type=int, default=2)
    parser.add_argument("--point-step", type=int, default=3)
    parser.add_argument("--horizontal-fov-deg", type=float, default=70.0)
    parser.add_argument("--near-m", type=float, default=0.5)
    parser.add_argument("--far-m", type=float, default=5.0)
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument("--cpu", action="store_true")
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    estimator = DepthEstimator(
        args.model, not args.cpu, args.near_m, args.far_m
    )
    for source in sorted(args.inputs):
        convert(
            source.resolve(),
            args.output_root.resolve(),
            estimator,
            max(1, args.frame_stride),
            max(1, args.point_step),
            args.horizontal_fov_deg,
            args.jpeg_quality,
        )


if __name__ == "__main__":
    main()
