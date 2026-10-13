# ROSを使ったPoint Cloud処理

この教材では、ROSにおける **Point Cloud（点群）データ**について入門的に学びます。

また、簡単な課題として、

**移動ロボットに搭載されたRGB-Dカメラの前を動いている人物を追跡する**

プログラムを作成します。

---

# 0. 追加データの準備

## 0.1 Rosbagデータ

まず、用意されている追加データをダウンロードしておいてください。

ダウンロードしたフォルダは、以下のディレクトリに配置します。

```bash
~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/
```

正しく配置できているか確認してみましょう。

1つ目のターミナルで、次のコマンドを実行します。

```bash
ros2 bag play ~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/person_static --loop
```

次に、2つ目のターミナルで以下を実行します。

```bash
ros2 topic list
```

すると、複数のROS topicが表示されるはずです。

# ROSを使ったPoint Cloud処理

この教材では、ROSにおける **Point Cloud（点群）データ**について紹介します。

また、簡単な課題として、

**移動ロボットに搭載されたRGB-Dカメラの前を動いている人物を追跡する**

プログラムを作成します。

---

# 0. 追加データの準備

## 0.1 Rosbagデータ

まず、用意されている追加データをダウンロードしておいてください。

ダウンロードしたフォルダは、以下のディレクトリに配置してください。

```bash
~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/
```

正しく配置できているか確認してみましょう。

1つ目のターミナルで次を実行します。

```bash
ros2 bag play ~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/person_static --loop
```

次に、2つ目のターミナルで以下を実行します。

```bash
ros2 topic list
```

すると、複数のROS topicが表示されるはずです。

---

# 1. RGB-D画像について理解する

このレッスンでは、これまでの2つのユニットで学習してきた内容をすべて利用します。

まずは `rgbd_reader.py` のコードを確認してみましょう。

main関数を見ると、これまでと同じように、最初にnodeを初期化しています。

```python
super().__init__("point_cloud_centroid")
```

その後、これから使用するtopicをsubscribeします。

```python
self.rgb_subscriber_ = self.create_subscription(
    Image, "/camera/rgb/image_rect_color", self.callback_rgb_rect, 10)
self.depth_subscriber_ = self.create_subscription(
    Image, "/camera/depth_registered/image", self.callback_depth_rect, 10)
self.point_cloud_subscriber_ = self.create_subscription(
    PointCloud2, "/camera/depth_registered/points", self.callback_point_cloud, 10)
```

ここでは、subscriberのcallback関数を使って、それぞれ異なる頻度で変数を更新し、それとは別にtimer関数を使って取得した情報を処理することの重要性が分かります。

今回は同じRGB-Dセンサから送られてくる3つのtopicをsubscribeしています。

- **RGB画像**
- **Depth画像**
- **3D点群であるPoint Cloud**

です。

---

RGB画像は、Red、Green、Blueの3チャンネルから構成されるカラー画像です。

一方、Depth画像は、画像の各pixelについて、そこに映っている物体がカメラ中心を通る直交平面からどれくらい離れているかをメートル法で表したものです。

これは、上から見たときに、ある点がカメラから水平方向にどれくらい離れているか、と考えることができます。

この距離はその点の高さには依存しません。

---

RGB画像については、これまでも `Image` topicを利用してきました。

Depth画像は、各pixelに対応する距離をミリメートル単位のfloatとして持つ行列です。

したがって、`callback_depth_rect` の中では次のように読み込みます。

```python
self.depth_=bridge_depth.imgmsg_to_cv2(msg,"32FC1").copy()
self.depth_mat_ = np.array(self.depth_, dtype=np.float32)
```

値の範囲は400、つまり40cmから、10000、つまり10mまでです。

そのため、画像として表示できる値の範囲へ正規化し、表示用のimage arrayとして保存します。

```python
cv2.normalize(self.depth_mat_, self.depth_, 0, 1, cv2.NORM_MINMAX)
```

**重要：**

Depth情報はstructured infrared light sensor（構造化赤外光センサ）から取得されています。

そのため、非常に反射率の高い表面や透明な表面ではDepth情報が歪む傾向があります。

そのような点はDepth画像上では黒、つまり0のpixelとして表示されます。

また、今回使用するRGB-Dセンサが読み取れる最短距離は40cmです。

したがって、40cmより近い物体についても値は0になります。

---

さらに、RGB画像とDepth画像を組み合わせることで、画像上の**すべてのpixelについて、実空間におけるXYZ座標を取得できます。**

ただし、幸いなことに、この計算について深く扱う必要はありません。

ROSがすでに計算してくれているからです。

`PointCloud2` 型の

```text
/camera/depth_registered/points
```

というtopicから、この情報を取得できます。

ターミナルで、

```bash
ros2 interface show sensor_msgs/msg/PointCloud2
```

と入力すると、このmessage typeがどのような構造になっているか確認できます。

---

Point Cloud messageは、

```text
(x, y, z, ...)
```

というtupleのリストとして3D点の情報を持っており、型は `PointCloud2` です。

そのため、`callback_point_cloud` では次のように読み込みます。

```python
self.point_cloud_ = np.array(
    list(pc2.read_points(msg, field_names=None, skip_nans=False))
)
```

これによってPoint Cloudはlistとして読み込まれます。

そこで、RGB画像と対応するmatrix形式になるようreshapeします。

```python
if msg.height > 1:
    self.point_cloud_ = self.point_cloud_.reshape(
        (msg.height, msg.width, -1)
    )

    rows, cols, _ = self.point_cloud_.shape
    print(
        'new message has arrived; point cloud size: rows: {}, cols: {}'
        .format(rows, cols)
    )
```

---

コードに戻ると、カメラの前に存在するobjectの3D位置をpublishするためのROS publisherがあります。

topicには一意な名前を付ける必要があります。

今回は、

```text
/object_centroid
```

という名前で、型は `Pose` としています。

```python
self.centroid_publisher_ = self.create_publisher(
    Pose, "/object_centroid", 10)
```

`Pose` messageは `geometry_msgs` の型です。

これは、カメラ中心を基準として、空間上のある対象について、

- 3次元位置
- quaternion形式の4次元orientation

を保持します。

具体的には、

```text
object_centroid.position.x
object_centroid.position.y
object_centroid.position.z

object_centroid.orientation.x = quaternion[0]
object_centroid.orientation.y = quaternion[1]
object_centroid.orientation.z = quaternion[2]
object_centroid.orientation.w = quaternion[3]
```

となります。

---

`PointCloud2` messageにおける座標軸は次のようになっています。

- **x:** カメラ中心から右方向が正
- **y:** カメラ中心から下方向が正
- **z:** カメラ中心から前方向が正

カメラ中心を原点とした場合、

- XY平面を正面から見たものと、その平面上に存在する点 `p` の **ROLL**
- YZ平面を横から見たものと、その平面上に存在する点 `p` の **PITCH**
- XZ平面を上から見たものと、その平面上に存在する点 `p` の **YAW**

は、それぞれREADME中の図のようになります。

---

### Homework 1.1

`rgbd_reader.cpp` の実装を確認してください。

C++版とPython版の構造は非常によく似ています。

したがって、自分の要件に応じて、どちらを使用しても構いません。

---

# 2. Point Cloudの操作

## 2.1 単一要素へのアクセス

ここから、`point_cloud_processing` というtimer関数の中で、取得した情報を処理していきます。

Pythonでは、

```python
self.processing_timer_ = self.create_timer(
    0.030,
    self.point_cloud_processing
)
```

となります。

C++では、

```cpp
processing_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(
        &PointCloudCentroidNode::point_cloud_processing,
        this
    )
);
```

となります。

---

Pythonでは、普通のarrayと同じように、

```python
self.point_cloud_[row_id, col_id, 0]
```

とすることで、array内の1要素へアクセスできます。

さらにX、Y、Zのいずれか1つの次元へアクセスしたい場合は、

```python
self.point_cloud_[row_id, col_id, 0][XYZ]
```

と直接指定できます。

ここで、

```text
XYZ = 0 → X
XYZ = 1 → Y
XYZ = 2 → Z
```

です。

今回の例では、画像中央にある点の3D情報を取得するために次のようにします。

```python
rows, cols, _= self.point_cloud_.shape

row_id = int(rows/2)
col_id = int(cols/2)

p = [
    float(self.point_cloud_[row_id, col_id, 0][0]),
    float(self.point_cloud_[row_id, col_id, 0][1]),
    float(self.point_cloud_[row_id, col_id, 0][2])
]
```

---

C++ではmatrix内の要素へ、

```cpp
point_cloud_.at<cv::Vec4f>(row_id, col_id)
```

としてアクセスします。

個々のcomponentへは、

```cpp
point_cloud_.at<cv::Vec4f>(row_id, col_id)[XYZ]
```

としてアクセスできます。

Python版と同様、

```text
XYZ = 0 → X
XYZ = 1 → Y
XYZ = 2 → Z
```

です。

例えば、

```cpp
int rows = point_cloud_.rows;
int cols = point_cloud_.cols;

int row_id = rows / 2;
int col_id = cols / 2;

cv::Vec4f point =
    point_cloud_.at<cv::Vec4f>(row_id, col_id);
```

となります。

---

最後に、この値をglobal variableへ保存し、後ほどprogramのmain loopにおいてROS publisherからpublishします。

Pythonでは、

```python
self.centroid_=Pose()

self.centroid_.position.x = p[0]
self.centroid_.position.y = p[1]
self.centroid_.position.z = p[2]

self.centroid_.orientation.x=0.0
self.centroid_.orientation.y=0.0
self.centroid_.orientation.z=0.0
self.centroid_.orientation.w=1.0

# Publish centroid pose
self.centroid_publisher_.publish(self.centroid_)
```

となります。

C++では、

```cpp
geometry_msgs::msg::Pose centroid;

centroid.position.x = static_cast<float>(point[0]); // x
centroid.position.y = static_cast<float>(point[1]); // y
centroid.position.z = static_cast<float>(point[2]); // z

centroid.orientation.x = 0.0;
centroid.orientation.y = 0.0;
centroid.orientation.z = 0.0;
centroid.orientation.w = 1.0;

// Publish the centroid pose
centroid_publisher_->publish(centroid);
```

となります。

---

## 2.2 コードを実行する

それではコードを実際に動かしてみましょう。

以下のコマンドを実行します。

```bash
cd ~/robovision_ros2_ws
colcon build
```

RGB-Dカメラを持っていなくても問題ありません。

Turtlebot 2の上部、高さ1.0mの位置に取り付けたXTion Proから収集したデータを含むROS bagが用意されています。

別のターミナルで、

```bash
ros2 bag play ~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/person_static --loop
```

を実行します。

その後、さらに別のターミナルで、

```bash
ros2 topic list
```

と入力します。

利用できるさまざまなtopicが表示されているでしょうか？

---

Python実装を使用する場合は、

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_rgbd rgbd_reader.py
```

と入力します。

C++実装の場合は、

```bash
source ~/robovision_ros2_ws/install/setup.bash
ros2 run robovision_rgbd rgbd_reader
```

と入力します。

画像中央にある点の3D情報を確認できるでしょうか？

---

最後に、新しいターミナルを開いて、

```bash
ros2 topic info /object_centroid
```

を実行します。

その後、

```bash
ros2 topic echo /object_centroid
```

を実行します。

すると、先ほどと同じ情報がROS topic上にpublishされていることを確認できるはずです。

---

### Homework 2.1

**1.**

画像上の異なる5つの点について、3D位置を取得してください。

異なる `row_id` と `col_id` を指定してみましょう。

画像中央の点を基準として、X、Y、Zの値がどのように変化するか確認できますか？

- Xは画像中央から**右方向が正**
- Yは画像中央から**下方向が正**

です。

---

**2.**

```text
row_id = 0
col_id = 0
```

の点について、3D情報はどうなっているでしょうか？

structured lightの反射特性などによって、ある点の情報を取得できない場合があります。

その場合、systemは、

```text
nan
```

という値を返します。

Pythonでは、`math` libraryの、

```python
math.isnan()
```

を使うことで、あるvariableが `nan` かどうか確認できます。

この関数は `True` または `False` を返します。

例えば、

```python
if (not math.isnan(
    self.point_cloud_[row_id, col_id, 0][0]
)):
```

のような構造を使ってdataを検証できます。

同様にC++では、

```cpp
std::isnan()
```

を使用できます。

---

# 3. 最終プロジェクト

これで、面白いrobot vision projectを作るために必要な道具はすべて揃いました。

今回解決したい問題は次のものです。

ロボットが、移動している人物の方向を向いているとします。

そこで、

**この人物がロボットの中心に対してどこにいるのか、その相対位置を求めたい**

とします。

ここではロボットの中心を、カメラの位置によって表現します。

言い換えると、

**`person_1` の3D位置を求めたい**

ということです。

では、どうすればよいでしょうか？

RGB-Dカメラを持っていない場合は、

```text
~/robovision_ros2_ws/src/robovision_ros2/data/rosbags/
```

にある、

```text
person_static
person_dynamic
```

というROS bagを利用できます。

---

## ヒント

Point Cloudには、カメラから見えている**すべての点の3D情報**が含まれていることを思い出してください。

それなら、その範囲を限定してしまうのはどうでしょうか？

余計な情報を取り除き、人物をより正確に追跡するための**有効領域**を作ることができます。

つまり、3D空間の中にboxを作り、そのbox内部だけを処理対象にできます。

---

画像内に存在するすべての3D点について、

**その点がこのboxの中に入っているかどうか**

を比較するような処理を作ることができます。

ただし、最初にその点が `nan` ではないことを確認してください。

---

適切な符号を使うことも忘れないでください。

特に **Y軸** に注意してください。

例えばカメラが床から1.0mの高さに設置されている場合、

**床そのものではない点は、どのような条件を満たす必要があるでしょうか？**

もう一度、Y軸の向きに特に注意してください。

---

## ヒント

`ROLL`、`PITCH`、`YAW` の定義を思い出してください。

これらの値はquaternion形式へ変換できます。

Pythonでは、

```python
from tf_transformations import quaternion_from_euler

quaternion = quaternion_from_euler(
    roll,
    pitch,
    yaw
)
```

とします。

そして、その値をvariableへ保存します。

```python
object_centroid.orientation.x = quaternion[0]
object_centroid.orientation.y = quaternion[1]
object_centroid.orientation.z = quaternion[2]
object_centroid.orientation.w = quaternion[3]
```

C++でも同様に、

```cpp
tf2::Quaternion quaternion;
quaternion.setRPY(roll, pitch, yaw);
```

として、

```cpp
centroid.orientation.x = quaternion.x();
centroid.orientation.y = quaternion.y();
centroid.orientation.z = quaternion.z();
centroid.orientation.w = quaternion.w();
```

とします。

それでは、Python版とC++版の両方を実装してみてください。

Good luck!

---

# Challenge 3.1

どうすればperformanceをさらに改善できるでしょうか？

**重要：各2D画像上の点には、Point Cloud内に対応する3D点が存在することを思い出してください。**

この関係を利用できます。

例えば、最初に人物の**顔を検出**して、その領域内に存在する3D点の平均を取る方法を試してみてもよいでしょう。

OpenCVには、このような処理に役立つさまざまな関数が用意されています。

例えばCascade Classifierがあります。

```text
https://docs.opencv.org/3.4/db/d28/tutorial_cascade_classifier.html
```

---

では、人物の顔だけではなく、**身体全体を検出**するのはどうでしょうか？

そして、検出された人物のbounding box内に存在する3D点の平均を取得します。

先ほどの例に、次のようなclassifierを追加できます。

```python
bodydetection = cv2.CascadeClassifier(
    'cascades/haarcascade_fullbody.xml'
)
```

このChallengeに挑戦してみますか？

---

# Authors

- **Luis Contreras** — ARTenshi
- **Hiroyuki Okada** — AIBot
