#!/usr/bin/env python3
"""Convert video files to independent ROS 2 bags of CompressedImage messages."""

import argparse
import re
import shutil
from datetime import datetime
from pathlib import Path

import cv2
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage


RECORDING_TIME = re.compile(
    r"Recording from (?P<date>\d{4}-\d{2}-\d{2}) "
    r"(?P<time>\d{2}-\d{2}-\d{2})\.(?P<fraction>\d+)"
)


def start_time_ns(video: Path) -> int:
    match = RECORDING_TIME.search(video.stem)
    if not match:
        return video.stat().st_mtime_ns

    timestamp = datetime.strptime(
        f"{match['date']} {match['time']}", "%Y-%m-%d %H-%M-%S"
    ).astimezone()
    fraction_ns = int(match["fraction"][:9].ljust(9, "0"))
    return int(timestamp.timestamp()) * 1_000_000_000 + fraction_ns


def convert(
    video: Path, output_root: Path, output_name: str, topic: str, quality: int
) -> Path:
    output = output_root / output_name
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")

    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open video: {video}")

    fps = capture.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        capture.release()
        raise RuntimeError(f"video has no valid frame rate: {video}")

    output_root.mkdir(parents=True, exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=topic,
            type="sensor_msgs/msg/CompressedImage",
            serialization_format="cdr",
            offered_qos_profiles="",
        )
    )

    first_stamp = start_time_ns(video)
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if not ok:
            capture.release()
            raise RuntimeError(f"failed to encode frame {frame_index}: {video}")

        stamp_ns = first_stamp + round(frame_index * 1_000_000_000 / fps)
        message = CompressedImage()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.header.frame_id = "camera"
        message.format = "jpeg"
        message.data = encoded.tobytes()
        writer.write(topic, serialize_message(message), stamp_ns)
        frame_index += 1

    capture.release()
    del writer  # Finalizes metadata.yaml before renaming the generated database.
    databases = list(output.glob("*.db3"))
    if len(databases) != 1:
        raise RuntimeError(f"expected one database in {output}, got {len(databases)}")
    old_database_name = databases[0].name
    databases[0].rename(output / "data.db3")
    metadata = output / "metadata.yaml"
    metadata.write_text(
        metadata.read_text().replace(old_database_name, "data.db3")
    )
    shutil.copy2(video, output / "video.webm")
    print(f"{video.name} -> {output} ({frame_index} frames at {fps:g} fps)")
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("videos", nargs="+", type=Path)
    parser.add_argument("--output-root", type=Path, default=Path("data/rosbags"))
    parser.add_argument("--topic", default="/camera/image/compressed")
    parser.add_argument("--jpeg-quality", type=int, default=90, choices=range(1, 101))
    parser.add_argument("--prefix", default="video")
    args = parser.parse_args()

    for index, video in enumerate(sorted(args.videos), start=1):
        convert(
            video.resolve(),
            args.output_root.resolve(),
            f"{args.prefix}_{index:02d}",
            args.topic,
            args.jpeg_quality,
        )


if __name__ == "__main__":
    main()
