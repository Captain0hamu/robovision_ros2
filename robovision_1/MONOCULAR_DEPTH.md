# 単眼動画からの推定Depth・疑似PointCloud

Ubuntuの通常カメラで撮影したWebMには実測Depthがありません。このノードは
Depth Anything Smallを使って相対的な奥行きを推定し、可視化用の疑似距離へ変換します。
距離計測、衝突回避、安全制御には使用しないでください。

## リアルタイム推定ノードの出力

| トピック | 型 | 内容 |
|---|---|---|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | RGB映像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 推定Depth、`32FC1` |
| `/camera/rgb/camera_info` | `sensor_msgs/CameraInfo` | 仮想ピンホールカメラ情報 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 色付き疑似PointCloud |

## 作成済みbagの再生（推奨・AI推論なし）

ローカルの `data/rosbags/video_01`～`video_04` と `hoodie_01`～`hoodie_04` には、
推定結果が保存されています。公開repoにbag実体は含まれません。
再生時にDepth AnythingのGPU推論は行いません（RViz描画は別です）。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robovision_images
source install/setup.bash

ros2 launch robovision_images play_depth_bag.launch.py \
  bag:=$PWD/data/rosbags/video_01
```

PointCloudのRViz表示サイズは5ピクセルです。

作成済みbagでは容量削減のためRGBとDepthを圧縮保存しています。

| トピック | 型 |
|---|---|
| `/camera/rgb/image/compressed` | `sensor_msgs/CompressedImage` |
| `/camera/depth/image/compressed` | `sensor_msgs/CompressedImage`（16UC1 PNG） |
| `/camera/rgb/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` |

## 動画からリアルタイム推定する場合

新しい動画を保存せず試す場合にだけ使用します。モデルは初回起動時に
Hugging Faceからダウンロードされ、以降はキャッシュが使われます。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robovision_images
source install/setup.bash

ros2 launch robovision_images monocular_depth.launch.py \
  video_path:=$PWD/data/rosbags/video_01/video.mp4
```

RVizを起動しない場合:

```bash
ros2 launch robovision_images monocular_depth.launch.py \
  video_path:=$PWD/data/rosbags/video_01/video.mp4 rviz:=false
```

GPUが使えない場合は自動的にCPUへ切り替わります。CPU推論を明示する場合:

```bash
ros2 run robovision_images monocular_depth_publisher.py --ros-args \
  -p video_path:=$PWD/data/rosbags/video_01/video.mp4 \
  -p use_gpu:=false
```

## 主なパラメータ

- `near_m` / `far_m`: 相対Depthを割り当てる表示用距離。既定値は0.5～5.0 m
- `horizontal_fov_deg`: カメラの仮定水平画角。既定値は70度
- `point_step`: PointCloudに使う画素間隔。小さいほど密で重くなる
- `frame_stride`: 推論時に読み飛ばすフレーム間隔
- `loop`: 動画末尾から繰り返すか

単眼推定には絶対スケールがないため、`near_m` と `far_m` は実距離の校正値ではありません。

## 新しい推定済みbagを作る場合

`tools/video_to_depth_bag.py` を使います。既定では30 fps動画を15 fpsへ間引き、
RGB・2 cm単位のDepth・CameraInfo・102,480点のPointCloudを同期保存します。

```bash
python3 tools/video_to_depth_bag.py ~/Videos/Camera/video_01.mp4 \
  --output-root data/output_bags
```
