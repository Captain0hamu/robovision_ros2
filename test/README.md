# 圧縮RGB・Depthから色付き点群を作るC++サンプル

このフォルダはROS 2 Humble用の独立したamentパッケージ `rgbd_cloud_demo`。
`image_transport::SubscriberFilter` で圧縮画像を内部で展開し、
RGB・Depth・RGB CameraInfoをApproximateTimeで同期してPointCloud2を生成する。
外部の `image_transport republish` と `depth_image_proc` の起動は不要。

## 入出力

| 入力 | 形式 |
|---|---|
| /head_rgbd_sensor/rgb/image_rect_color/compressed | 圧縮RGB |
| /head_rgbd_sensor/depth_registered/image/compressedDepth | 圧縮Depth |
| /head_rgbd_sensor/rgb/camera_info | CameraInfo |
| 出力 /head_rgbd_sensor/depth_registered/demo_points | PointCloud2 (XYZRGB) |

`src/rgbd_cloud.cpp` のsubscriberには末尾を除いたbase topicとtransport名を渡す。
自分のノード内で展開するため、展開後のraw topicは配信しない。
5秒ごとのログで展開済み画像・CameraInfoの受信数と点群の出力数を確認できる。

## ビルド（既存Docker内、リポジトリのルートから）

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rgbd_cloud_demo
source install/setup.bash
```

必要な依存は `rclcpp`, `sensor_msgs`, `image_transport`,
`message_filters`, `cv_bridge`, OpenCV、圧縮画像の2プラグイン、RViz2。
既存コンテナにない場合は対応するROS Humbleパッケージをインストールする。

## 起動

bagとノードは同じROS_DOMAIN_ID/RMW_IMPLEMENTATIONを使用する。
今回の `/head_rgbd_sensor/...` が収録されたbagを別タブで再生する。

```bash
ros2 bag play data/rosbags/person_rgbd_1 --clock
```

ノードと設定済みRViz2を一括起動する。

```bash
source install/setup.bash
ros2 launch rgbd_cloud_demo view.launch.py
```

GUIを起動せず点群生成だけを行う場合：

```bash
ros2 launch rgbd_cloud_demo view.launch.py rviz:=false
```

パラメータを指定してノードを直接起動する場合：

```bash
ros2 run rgbd_cloud_demo rgbd_cloud --ros-args \
  -p use_sim_time:=true \
  -p sync_queue_size:=30 \
  -p max_sync_interval_ms:=50.0
```

bag再生はノード起動後に始めると冒頭のデータも受信できる。
終了したbagは再生し直す。ループではtimestampが戻る警告が出ることがある。

## 確認・調整

Humbleの圧縮Depthプラグインでは起動時に
`SubscriberPlugin::subscribeImpl with five arguments has not been overridden`
と出る場合がある。旧形式の購読へ進む互換処理のログであり、今回の実データ検証では
このログ後も画像展開・点群生成が動作した。受信数と実際の点群出力で判断する。

```bash
ros2 node info /rgbd_cloud_demo
ros2 topic echo /head_rgbd_sensor/depth_registered/demo_points \
  --once --field header --qos-reliability best_effort
```

RViz2はPointCloud2/RGB8/Best Effortで表示する。初期Fixed Frameは
`head_rgbd_sensor_rgb_frame`。TFがあるなら `base_link` や `odom` など、
bagに実在するframeへ変更できる。

同期の最大timestamp区間は50ms、queueは30。
受信数だけ増えて点群が出ない場合は、timestamp差とCameraInfoの配信を確認する。
queueを大きくすると待機可能な到着ずれは増えるがメモリ消費も増える。
許容時間を大きくすると撮影時刻が離れた画像を組み合わせる可能性がある。
ApproximateTimeはtimestamp完全一致や遅延ゼロを保証しない。

## 計算の前提

DepthはRGBに位置合わせ済み、RGBは歪み補正済みであることを前提とする。
このプログラム自体は位置合わせや歪み補正を行わない。
RGB/Depth/CameraInfoの解像度とframe_idが一致しない場合は処理をスキップする。
解像度の一致だけで位置合わせが保証されるわけではない。

Depthの16UC1はmmとしてmへ変換、32FC1はmとして扱う。
RGB CameraInfoの投影行列Pからfx/fy/cx/cyを読み、
`X=(u-cx)*Z/fx, Y=(v-cy)*Z/fy` でXYZを計算する。
座標はカメラの投影モデルに従う。既存TFを使ってRViz2で別frameへ変換できる。
距離0・NaN・無限大は無効な点としてXYZをNaNにする。
Depthの正規化や表示用の色変換は距離計算に使用しない。

## 検証結果

既存Docker（ROS 2 Humble）でビルド成功。
専用ROS domainでperson_rgbd_1を再生し、外部republish/depth_image_procなしで
640×480のXYZRGB点群を20フレーム受信した。frame_id、データサイズ、XYZ/RGBフィールド、
有効なXYZ座標が含まれることも確認した。同期timestamp区間の一例は9.940ms。
付属設定でRViz2がOpenGLを初期化して起動することも確認した。
検証プロセスは終了済み。RViz2画面上の点群の見え方は利用時に確認する。
