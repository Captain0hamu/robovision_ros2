# 同期RGBの顔検出と色付き点群の生成

RGB・Depth・RGB CameraInfoをApproximateTimeで同期し、同じcallbackで
顔検出画像の表示とPointCloud2の生成・配信を行う。

```text
/camera/rgb/image_rect_color ─┐
/camera/depth_registered/image ─┼→ callback_rgb_depth
/camera/rgb/camera_info ───────┘       ├→ create_point_cloud → XYZRGBを生成
                                     ├→ show_rgb → 顔検出・緑枠・imshow
                                     └→ publish → RViz2

最新の生成点群 → 既存30msタイマー → 5サンプル点の /object_centroid
```

独立したRGB表示callbackは廃止した。
bag内の `/camera/depth_registered/points` は購読しない。
Depthの表示も行わない。この同期ノードは既存の `rgbd_reader.cpp` とは別実装。
`rgbd_reader.cpp` 自体には過去の別実験の変更があるため、新規利用は同期ノードを推奨する。

## 起動（Docker内、リポジトリ直下）

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robovision_rgbd
source install/setup.bash
ros2 launch robovision_rgbd rgbd_reader_sync.launch.py
```

別タブで同じROS環境から再生する。

```bash
ros2 bag play data/rosbags/person_static --clock --loop
# または
ros2 bag play data/rosbags/person_dynamic --clock --loop
```

ROS_DOMAIN_ID/RMW_IMPLEMENTATIONなどを合わせる。
launchは既定でuse_sim_time=true。bagは別途再生する。
GUIにはDockerのDISPLAY/X11接続が必要。
RVizを起動しない場合は `rviz:=false` を付ける。
元のrgbd_readerもnode名がpoint_cloud_centroidなので同時起動しない。

## 入力と出力

| 用途 | topic / type |
|---|---|
| RGB入力 | /camera/rgb/image_rect_color / Image（raw） |
| Depth入力 | /camera/depth_registered/image / Image（raw） |
| 校正入力 | /camera/rgb/camera_info / CameraInfo |
| 生成点群出力 | /camera/depth_registered/synced_points / PointCloud2 |
| 従来のサンプル点出力 | /object_centroid / Pose |

今回はraw画像を購読する。test/rgbd_cloud_demoの圧縮展開処理は不要。
点群生成の計算方式はtestと共通で、RGB・Depthから自分でXYZRGBを作る。

## 同期callback内の順番

1. RGBを描画可能なBGRコピー、Depthを元の値を保持したMatにする。
2. 空画像・サイズ・Depth encoding・CameraInfo.P・frame_idを検証する。
3. `create_point_cloud()` で、枠を描く前のRGBとDepthから点群を作る。
4. `show_rgb()` で同じRGBへ顔の緑枠を描き、ウィンドウを更新する。
5. 生成した点群をpublishし、タイマー用の最新点群として保持する。

同じ同期組を使うため、RGB表示だけが先のフレームへ進む経路はない。
ただしApproximateTimeは完全に同じ撮影時刻を保証しない。
3入力の最大timestamp区間は既定50msで、queueは100。
出力PointCloud2のheaderはDepthのheaderを引き継ぐ。
debugログの `Sync stamps: rgb=... depth=... info=...` で使用した組を確認できる。

OpenCVのウィンドウ更新と、通信・描画を経由するRViz2の更新は別々なので、
画面が同じ瞬間に描き替わることまで保証しない。
同期によって処理遅延がなくなるわけではない。
RVizは最新の生成点群を表示する設定である。

person_staticのDepthは32枚、person_dynamicは68枚。
生成回数は同期に使えたDepth枚数以下となる。
RGBの353/943枚すべてに対応する点群が作れるわけではない。
bagに記録された点群6/8件より多い点群を生成できるが、滑らかな30fpsは保証しない。
bagの到着時刻とheaderの時刻には秒単位のずれがあるため、
大きめのqueueで待つ。末尾の未確定の組が出力されないこともある。
ループ再生ではtimestampが戻る警告が出る場合がある。

## 点群の計算

RGBは歪み補正済み、DepthはRGBへ位置合わせ済みであることが前提。
画像サイズ・frame_idの一致だけで位置合わせの正しさは証明できない。
このコードで位置合わせや歪み補正を行うわけではない。

Depthの16UC1はmmとして0.001倍してmへ変換する。
32FC1はmのまま使用する。距離値を表示用に正規化しない。
CameraInfoの投影行列Pからfx/fy/cx/cyを取り出して計算する。

```text
Z = depth（m）
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

同じ画素のBGRを点群のRGBへ格納する。
距離0・負値・NaN・無限大はXYZをNaNにする。
width/heightを画像と同じに保つorganized cloud。
640×480の入力なら307,200点の配置となる。
顔枠を描く前に色をコピーするため、緑の線は点群の色に混ざらない。

## 顔検出と従来のPose出力

CascadeClassifierはprivateメンバで、XMLはコンストラクタで一度ロードする。
`config/haarcascade_frontalface_default.xml` はshareへinstallする。
別のモデルは `-p face_cascade_path:=/絶対パス/model.xml` で指定できる。

顔枠は2D表示のみ。顔の点群切り出しや顔の3D座標は計算しない。
正面顔向けのHaar Cascadeなので見逃しや誤検出はあり得る。

`point_cloud_processing()` は従来どおり30msごとに5つの画素を読み、
最新の生成点群のXYZをPoseとして配信する。姿勢は単位Quaternion。
元のcv::Matへの詰め直しは廃止し、PointCloud2ConstIteratorでXYZを読む。
Poseにはtimestampやframe_idがなく、顔の重心でもない。
新しい入力がなければ同じ最新サンプル値を繰り返す従来の動作である。

## RViz2と調整

RViz2のtopicは `/camera/depth_registered/synced_points`。
Fixed Frameは `camera_rgb_optical_frame`、色はRGB8、QoSはBest Effort。
Move CameraとViewsパネルで視点を操作できる。
TFがあればFixed Frameをodomなどへ変更できる。

パラメータを調整する場合はノードを直接起動する。

```bash
ros2 run robovision_rgbd rgbd_reader_sync --ros-args \
  -p use_sim_time:=true \
  -p sync_queue_size:=100 \
  -p max_sync_interval_ms:=50.0 \
  --log-level point_cloud_centroid:=debug
```

関係するファイル：
src/rgbd_reader_sync.cpp、config/haarcascade_frontalface_default.xml、
launch/rgbd_reader_sync.launch.py、rviz/rgbd_reader_sync.rviz。

## 実データ検証

既存Docker内でビルド成功（コンパイラ警告なし）。
専用ROS domainでRGB・Depth・CameraInfoだけを再生し、
bagのPointCloud2を入力せず、person_staticで23フレーム、
person_dynamicで58フレームの640×480 XYZRGB点群を受信した。
dynamicは検証時に2倍速で再生した。

出力headerと元Depth、debugログのRGB/CameraInfoを照合し、各組の時刻区間が50ms以内、
サンプル画素のXYZがDepthとPによる計算に一致、色が同じ組のRGBに一致することを確認した。
両bagで顔検出と既存Pose出力も確認した。
RVizが新しいsynced_pointsをBest Effortで購読することも確認した。
検証用プロセスは終了済みで、ユーザーの既存bag再生には変更していない。
