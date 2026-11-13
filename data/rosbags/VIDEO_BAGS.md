# 動画から作成したROS 2 bag

`video_01/`～`video_04/` と `hoodie_01/`～`hoodie_04/` には、
動画ごとにRGB・推定Depth・点群をまとめたROS bagを置いています。

- RGB: `/camera/rgb/image/compressed`（JPEG）
- 推定Depth: `/camera/depth/image/compressed`（16UC1 PNG、2 cm単位）
- CameraInfo: `/camera/rgb/camera_info`
- 色付き疑似PointCloud: `/camera/depth/points`（102,480点/フレーム）
- 収録レート: 15 fps
- `video.mp4`: 変換元動画のコピー
- bag内の時刻: MP4の更新時刻を起点に、15fpsの間隔で記録（撮影時刻とは限りません）

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

RGB・Depth・点群をRVizでまとめて表示する場合：

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

Dockerで動画からbagを作る例です。ホスト側の動画フォルダを読み取り専用でマウントします。

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

同名のbagがあると処理を止めます。作り直すときは、別の出力先を指定してください。

推論するのは、この変換を行うときだけです。作成したbagの再生中は推論しません。
