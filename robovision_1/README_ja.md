# Lesson 1：ROS 2で画像を送受信する

[English](README.md) · [全体のREADME](../README.md) · [次へ：画像処理](../robovision_2/README_ja.md)

ROS 2とOpenCVを使い、静止画やカメラ映像を別のノードへ送ります。
画像の配信、受信、表示を通して、トピックとコールバックの使い方を学びます。

このフォルダのROSパッケージ名は `robovision_images` です。
以下のコマンドは、リポジトリのルートから実行してください。

## 1. ビルドの準備

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robovision_images
source install/setup.bash
```

別のターミナルを開いたら、そちらでも環境を読み込みます。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

OpenCVのウィンドウを表示するため、GUIが使える環境が必要です。
DockerではDISPLAY/X11の設定に加え、カメラを使う場合はデバイスへのアクセスも必要です。

### package.xmlとCMakeLists.txt

`package.xml` には、パッケージ名や依存するライブラリを書きます。
この教材では `rclcpp`、`rclpy`、`sensor_msgs`、`cv_bridge` などを使います。

`CMakeLists.txt` では、C++のソースを実行ファイルにし、Pythonスクリプトとともにインストールします。
たとえば、`my_publisher` の設定は次の形です。

```cmake
add_executable(my_publisher src/my_publisher.cpp)
ament_target_dependencies(my_publisher rclcpp cv_bridge sensor_msgs OpenCV)
```

`install(TARGETS ...)` や `install(PROGRAMS ...)` で配置した実行ファイルを、
`ros2 run パッケージ名 実行ファイル名` で起動します。

## 2. 静止画を配信する

```bash
ros2 run robovision_images my_publisher "$PWD/data/images/baboon.png"
```

`src/my_publisher.cpp` は画像を一度読み込み、タイマーで繰り返し配信します。
このforkでは、元画像に加えて縦横を半分にした画像も配信しています。

| トピック | 内容 | 型 |
|---|---|---|
| `/camera/image` | 元画像 | `sensor_msgs/msg/Image` |
| `/camera/scaled_image` | 縦横を半分にした画像 | `sensor_msgs/msg/Image` |

画像を読み込むのは `cv::imread`、ROSメッセージへの変換は `cv_bridge` の役目です。

```cpp
cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
auto message = cv_bridge::CvImage(
    std_msgs::msg::Header(), "bgr8", image).toImageMsg();
```

OpenCVで読み込んだ画像は通常BGR順です。`bgr8` は、青・緑・赤の各成分を
8bitで持つ画像であることを表します。

配信側は、トピック名とメッセージ型を指定して作ります。

```cpp
image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    "camera/image", 5);
```

末尾の `5` はキューの深さです。fpsの指定ではありません。
タイマーの間隔は30msなので、処理が間に合えば約33Hzで同じ画像を送ります。

### トピックを確認する

別のターミナルで実行します。

```bash
ros2 topic list
ros2 topic info /camera/image
ros2 topic hz /camera/image
ros2 node info /image_publisher
```

型が `sensor_msgs/msg/Image` になっているか、配信ノードが見えるかを確認してください。

## 3. カメラ映像を配信する

静止画の配信を止めてから、次を実行します。どちらも同じトピックとノード名を使います。

```bash
ros2 run robovision_images my_camera_publisher --ros-args -p camera_index:=0
```

`src/my_camera_publisher.cpp` は `cv::VideoCapture` でカメラを開き、
タイマーが呼ばれるたびにフレームを読み込んで `/camera/image` へ送ります。

`camera_index` で使うカメラを選びます。番号は環境によって変わるため、
内蔵カメラが必ず0とは限りません。開けない場合は番号、権限、他アプリの使用状況を確認してください。
カメラの実際のfpsは、デバイスと読み込み処理にも左右されます。

## 4. 画像を受信して表示する

画像を配信したまま、別のターミナルで受信ノードを起動します。

```bash
# C++版
ros2 run robovision_images my_subscriber
```

Python版を試す場合は、C++版を止めてから起動してください。

```bash
ros2 run robovision_images my_subscriber.py
```

受信側では、画像が届くとコールバックが呼ばれます。
ここでROSメッセージをOpenCVの画像に変換し、メンバ変数へ保存します。

```cpp
void callback_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
    image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    is_image_ = true;
}
```

Pythonでは `CvBridge().imgmsg_to_cv2(...)` で変換します。
表示処理は別のタイマーで行い、`imshow` と `waitKey` でウィンドウを更新します。
`waitKey` はキー入力だけでなく、OpenCVの画面更新にも必要です。

```bash
ros2 node list
ros2 node info /image_subscriber
```

`/camera/image` を購読していることを確認してください。

### このforkでの変更

C++版は `/camera/scaled_image` も受信しますが、現在の表示処理では元画像だけを表示します。
縮小画像を別ウィンドウに表示する部分は、次の演習として追加できます。

## 5. 演習

1. 元画像と縮小画像の幅・高さを比較してください。画素数は何分の1になりますか？
2. カメラ配信にも縮小画像のトピックを追加してください。
   配信側を作るのは初期化時、縮小するのはフレームを読み込んだ後です。
3. 受信ノードで縮小画像も表示してください。元画像とは別のウィンドウ名を使います。
4. タイマーの間隔を変え、`ros2 topic hz` の結果を比べてください。

## 6. このforkで追加した動画・Depth機能

このレッスンの画像送受信とは別に、動画からDepthと点群を作る機能も追加しています。
使い方は [MONOCULAR_DEPTH.md](MONOCULAR_DEPTH.md) を参照してください。
通常のカメラ映像から作るDepthはAIによる推定であり、実測距離ではありません。

## 参考

- [ROS 2 Humble：C++のPublisher / Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [ROS 2 Humble：PythonのPublisher / Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

元教材の著者：**Luis Contreras**（[ARTenshi](https://artenshi.github.io/)）、
**Hiroyuki Okada**（[AIBot](http://aibot.jp/)）。日本語版はこのforkのコードに合わせて整理しています。
