# 単眼動画からの推定Depth・疑似PointCloud

通常のカメラアプリで撮影した動画には、Depth情報がありません。
Depth Anything Smallで奥行きを推定し、表示用の疑似距離に変換して点群を作ります。
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
推定結果を保存しています。GitHubにはbag本体を含めていないため、別途用意してください。
再生時にDepthの推論は行いませんが、RVizの描画にGPUを使うことはあります。

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

新しい動画で試したいときは、こちらを使えます。毎フレーム推論するため、再生中も負荷がかかります。
モデルは初回にHugging Faceからダウンロードし、次回からはキャッシュを使います。

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

GPUが使えない場合は自動でCPUに切り替わります。CPUで処理したい場合はこちら：

```bash
ros2 run robovision_images monocular_depth_publisher.py --ros-args \
  -p video_path:=$PWD/data/rosbags/video_01/video.mp4 \
  -p use_gpu:=false
```

## 主なパラメータ

- `near_m` / `far_m`: 表示上の近距離と遠距離。初期値は0.5～5.0 m
- `horizontal_fov_deg`: 点群の計算に使う仮の水平画角。初期値は70度
- `point_step`: PointCloudに使う画素間隔。小さいほど密で重くなる
- `frame_stride`: 推論時に読み飛ばすフレーム間隔
- `loop`: 動画末尾から繰り返すか

`near_m` と `far_m` を変えても、実際の距離に校正されるわけではありません。

## 新しい推定済みbagを作る場合

`tools/video_to_depth_bag.py` を使います。初期設定では30fpsの動画から1フレームおきに取り出し、
15fpsで保存します。RGB、2cm単位のDepth、CameraInfo、102,480点の点群を同じ時刻で記録します。

```bash
python3 tools/video_to_depth_bag.py ~/Videos/Camera/video_01.mp4 \
  --output-root data/output_bags
```
