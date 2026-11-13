# Lesson 3：RGB-D画像と点群を扱う

[English](README.md) · [前へ：画像処理](../robovision_2/README_ja.md) · [次へ：サービス](../robovision_4/README_ja.md)

RGB-Dカメラは、カラー画像に加えて奥行きの情報を取得します。
このレッスンではDepthと点群の関係を学び、画像上の点に対応する3D位置を読み取ります。
最後に、人物の位置を求める処理を考えます。

ROSパッケージ名は `robovision_rgbd` です。
元教材の説明に加え、このforkで追加した同期ノードの使い方もまとめています。

## 1. データとノードを選ぶ

元教材のbagは [元READMEの配布先](../docs/UPSTREAM_README.md#22-download-additional-data) から入手し、
`data/rosbags/` に配置してください。GitHubにはbag本体を含めていません。

| データ | このforkでの使い方 |
|---|---|
| `person_static` / `person_dynamic` | このレッスンの `rgbd_reader_sync` で、RGB・Depthから点群を生成 |
| `person_rgbd_1` | [rgbd_cloud_demo](../test/README.md) で圧縮RGB-Dを展開して点群を生成 |
| `video_*` / `hoodie_*` | [保存済みbag](../data/rosbags/VIDEO_BAGS.md) を再生。DepthはAI推定値 |

元の `rgbd_reader.cpp` は実験途中で、PointCloud2をCameraInfoのトピックから受け取ろうとするなどの問題があります。
これから試す場合は、以下の同期ノードを使ってください。
Python版 `scripts/rgbd_reader.py` は元教材の点群読み取りを学ぶために残しています。

## 2. ビルドして起動する

以下はリポジトリのルートから実行します。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robovision_rgbd
source install/setup.bash
ros2 launch robovision_rgbd rgbd_reader_sync.launch.py
```

別のターミナルで環境を読み込み、bagを再生します。
ノードを先に起動しておくと、冒頭のフレームも受信しやすくなります。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play data/rosbags/person_static --clock --loop
```

動きのあるデータで試すなら、`person_static` を `person_dynamic` に変えてください。
GUIにはOpenCVとRVizを表示できる環境が必要です。RVizだけを止めたい場合は
launchに `rviz:=false` を付けます。OpenCVの画像表示は残ります。

### 入出力を確認する

```bash
ros2 topic list
ros2 topic info /camera/depth_registered/synced_points
ros2 topic echo /object_centroid --once
```

| 用途 | トピック | 型 |
|---|---|---|
| RGB入力 | `/camera/rgb/image_rect_color` | `sensor_msgs/msg/Image` |
| Depth入力 | `/camera/depth_registered/image` | `sensor_msgs/msg/Image` |
| カメラ情報 | `/camera/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 生成した点群 | `/camera/depth_registered/synced_points` | `sensor_msgs/msg/PointCloud2` |
| サンプル点の位置 | `/object_centroid` | `geometry_msgs/msg/Pose` |

## 3. RGBとDepthの違い

RGB画像は画素ごとの色、Depth画像は画素ごとの奥行きを表します。
ここでいう奥行きは、カメラの光軸方向の距離Zです。カメラから点までの直線距離とは区別します。

このノードでは、Depthを次の単位として扱います。

| encoding | データ型 | 単位 |
|---|---|---|
| `16UC1` | 符号なし16bit整数、1チャンネル | mm。計算時に0.001倍してmへ変換 |
| `32FC1` | 32bit浮動小数点、1チャンネル | m |

実際に使うセンサーやデータの仕様も確認してください。
**表示用に0〜1へ正規化したDepthを、3D座標の計算に使ってはいけません。**
画面表示には正規化したコピーを使い、計算用の距離値はそのまま残します。

透明な物体、反射の強い面、センサーの測定範囲外などでは、Depthが得られないことがあります。
無効値はデータによって0やNaNなどで表されます。測定可能な距離もセンサーごとに異なります。

## 4. DepthからXYZを計算する

RGBとDepthを組み合わせるには、DepthがRGBの画素に位置合わせされている必要があります。
このノードは位置合わせや歪み補正を行いません。画像のサイズが同じだけでは、位置合わせ済みとは判断できません。

位置合わせ済みのDepthと、対応するCameraInfoの投影行列Pを使って計算します。

```text
Z = depth（m）
X = (u - cx) × Z / fx
Y = (v - cy) × Z / fy
```

`u` は列、`v` は行です。`fx`・`fy` は画素単位の焦点距離、`cx`・`cy` は画像上の主点を表します。
同じ画素のRGBをXYZと組み合わせると、色付き点群になります。

カメラの光学座標系では、Xは右、Yは下、Zは前が正です。
ロボット本体の座標系とは軸の向きが異なる場合があるので、点群の `header.frame_id` を確認してください。

## 5. PointCloud2を読む

点群の型は次のコマンドで確認できます。

```bash
ros2 interface show sensor_msgs/msg/PointCloud2
```

PointCloud2はXYZや色などをバイナリデータとして持ちます。
値の場所と型は `fields`、1点のバイト数は `point_step`、1行のバイト数は `row_step` で分かります。
データが常に「floatのXYZRGB順」とは限らないため、レイアウトを確認して読み取ります。

### 画像の画素と対応する点群

行・列の並びを保った点群をorganized cloudと呼びます。
同期ノードはこの形式で出力し、640×480画像なら307,200点の配置になります。
無効な点も位置を詰めずにNaNとして残すため、画像の画素との対応を保てます。

C++では、`sensor_msgs::PointCloud2ConstIterator<float>` で各フィールドを読み取れます。
次はXYZがfloat32で、行の末尾に余分なバイトがないorganized cloudから、中央の点を読む例です。

```cpp
const std::size_t row = cloud.height / 2;
const std::size_t col = cloud.width / 2;
const std::size_t index = row * cloud.width + col;
sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
x += index;
y += index;
z += index;
```

Pythonでは `sensor_msgs_py.point_cloud2.read_points` を使えます。
無効点を読み飛ばすと画素との対応が変わるため、画像の行・列で読むときは注意してください。
戻り値の配列形式はライブラリの版にもよるので、型とshapeを確認して扱います。

`video_*` や `hoodie_*` の点群は間引いた点を1列に並べた形式です。
画像の行・列を、そのまま点群の添字には使えません。

## 6. 位置をPoseで送る

同期ノードは中央とその上下左右の5点を読み、`/object_centroid` へ順番に配信します。
これは顔や人物の重心ではなく、画像内のサンプル点です。

```cpp
geometry_msgs::msg::Pose pose;
pose.position.x = *x;
pose.position.y = *y;
pose.position.z = *z;
pose.orientation.w = 1.0;
```

Poseは位置と姿勢を持ちますが、時刻や座標系名は持ちません。
必要なら `PoseStamped` などを使います。向きを指定しない場合は、
姿勢を `(x, y, z, w) = (0, 0, 0, 1)` にします。

点のXYZが分かっただけでは、対象の姿勢まで分かるわけではありません。
姿勢を別に求めた場合は、roll・pitch・yawからクォータニオンへ変換できます。

```cpp
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);
```

ROSのroll・pitch・yawは、それぞれX・Y・Z軸まわりの回転です。
カメラの光学座標系を使う場合も、軸の定義を変えずに考えてください。

## 7. このforkで追加した同期処理

`src/rgbd_reader_sync.cpp` はRGB・Depth・CameraInfoをApproximateTimeで組み合わせ、
点群生成と顔検出画像の表示を同じコールバックで処理します。
顔枠を描く前のRGBから点群の色を取るので、緑の枠は点群に混ざりません。

ただし、時刻が完全に一致する保証はありません。許容する時刻差は初期設定で50msです。
点群の更新回数はDepthの枚数に制限され、RGBと同じfpsになるとは限りません。
また、`/object_centroid` のタイマーは、新しい入力がなくても最新のサンプル値を繰り返し送ります。

詳細な設定と確認結果は [RGBD_SYNC.md](RGBD_SYNC.md) を参照してください。

## 8. 演習

1. 画像内の5か所についてXYZを読み、右・下・奥へ動くと各値がどう変わるか確認してください。
2. 画像の端などで無効値を探してください。C++なら `std::isfinite`、Pythonなら `math.isfinite` で確認できます。
3. RGB・Depth・CameraInfoの時刻を調べ、同期の許容幅を変えたときの受信数を比べてください。

### 人物の位置を求める

元教材の発展課題は、カメラに映った人物の相対位置を求めることです。

まず、XYZの範囲で3Dの箱を決め、その中にある有効点だけを選ぶ方法を考えてください。
床や遠い背景を除外しやすくなります。Y軸は下向きなので、床を除く条件の符号に注意します。
カメラが傾いている場合は、床の位置を高さだけでは判断できません。

次に、顔や身体の検出枠と点群を組み合わせてみてください。
検出枠内の有効点から平均や中央値を求める方法がありますが、背景の混入や誤検出への対策も必要です。
このforkの顔検出は2D表示だけで、顔の3D位置や追跡までは実装していません。

参考：[OpenCVのCascade Classifier](https://docs.opencv.org/4.x/db/d28/tutorial_cascade_classifier.html)

元教材の著者：**Luis Contreras**（[ARTenshi](https://artenshi.github.io/)）、
**Hiroyuki Okada**（[AIBot](http://aibot.jp/)）。日本語版はこのforkのコードに合わせて整理しています。
