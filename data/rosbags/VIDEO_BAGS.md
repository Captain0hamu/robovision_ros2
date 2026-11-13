# 動画から作成したROS 2 bag

`video_01/`～`video_04/` と `hoodie_01/`～`hoodie_04/` は、それぞれ1本の
MP4動画を独立したROS 2 bagへ変換したものです。

- RGB: `/camera/rgb/image/compressed`（JPEG）
- 推定Depth: `/camera/depth/image/compressed`（16UC1 PNG、2 cm単位）
- CameraInfo: `/camera/rgb/camera_info`
- 色付き疑似PointCloud: `/camera/depth/points`（102,480点/フレーム）
- 収録レート: 15 fps
- `video.mp4`: 変換元動画のコピー
- bag内の時刻: 開始はMP4更新時刻（撮影時刻とは限らない）、元30 fpsから15 fps分を抽出

| データ名 | 動画長 |
|---|---|
| `video_01` | 56.3秒 |
| `video_02` | 21.0秒 |
| `video_03` | 25.0秒 |
| `video_04` | 38.3秒 |
| `hoodie_01` | 19.4秒 |
| `hoodie_02` | 16.3秒 |
| `hoodie_03` | 29.4秒 |
| `hoodie_04` | 13.0秒 |

## 再生・確認

Dockerコンテナ内で、リポジトリのルートから実行します。

```bash
ros2 bag info data/rosbags/video_01
ros2 bag play data/rosbags/video_01 --loop
```

RGB・Depth・PointCloudを設定済みRVizで表示する例:

```bash
source install/setup.bash
ros2 launch robovision_images play_depth_bag.launch.py \
  bag:=$PWD/data/rosbags/video_01
```

raw画像トピックが必要な場合:

```bash
ros2 run image_transport republish compressed raw \
  --ros-args \
  -r in/compressed:=/camera/depth/image/compressed \
  -r out:=/camera/depth/image_raw
```

## 別の動画を変換する方法

この環境ではROS 2がDocker内にあるため、ホストの動画ディレクトリをマウントして
オフライン変換スクリプトを実行します。

```bash
cd ~/ros2_docker
docker run --rm --gpus all --entrypoint bash \
  -v "$PWD/robovision_ros2:/workspace" \
  -v "$HOME/Videos/Camera/mp4:/videos:ro" \
  tidbots/hsr-ros2:humble-nvidia -lc \
  'source /opt/ros/humble/setup.bash && cd /workspace && \
   python3 tools/video_to_depth_bag.py /videos/*.mp4 \
     --output-root data/output_bags'
```

出力先に同名のbagがある場合は、誤上書きを防ぐため変換を停止します。

この処理はGPU推論を一度だけ行うオフライン変換です。作成済みbagの再生時には
GPU推論しません。
