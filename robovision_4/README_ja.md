# Lesson 4：サービスとクライアントを使う

[English](README.md) · [前へ：RGB-Dと点群](../robovision_3/README_ja.md) · [全体のREADME](../README.md)

トピックは画像のようなデータを継続して送る仕組みです。
一方、サービスは「要求を送り、その結果を受け取る」ために使います。
このレッスンでは画像上の座標を指定し、その画素に対応する3D位置を問い合わせます。

パッケージ名は `robovision_services`、要求と応答の型は `robovision_interfaces` にあります。

## 1. メッセージとサービスの定義

`robovision_interfaces/msg/ObjectCentroid.msg` は、位置を格納するメッセージです。

```text
float64 x
float64 y
float64 z
float64[] centroid
```

この教材では、同じXYZを `x`、`y`、`z` と配列 `centroid` の両方に入れて返します。

`robovision_interfaces/srv/GetPointCenter.srv` は次の構成です。

```text
int64 x
int64 y
---
ObjectCentroid point
```

`---` より上が要求、下が応答です。
要求の `x` は画像の列、`y` は行を表します。応答のXYZは空間上の位置なので、混同しないようにしてください。
名前に「Center」や「Centroid」とありますが、現在の処理は**指定した1画素の3D位置**を返します。
物体全体の重心を計算するわけではありません。

### ビルドへの登録

独自の型は `rosidl_generate_interfaces` で生成します。

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectCentroid.msg"
  "msg/ObjectCentroids.msg"
  "srv/GetPointCenter.srv"
)
```

`package.xml` では型の生成と実行に必要な依存を宣言し、
利用側の `robovision_services` から `robovision_interfaces` を参照します。
詳しくは実際の両ファイルを確認してください。

## 2. ビルドする

以下はリポジトリのルートから実行します。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robovision_services
source install/setup.bash
```

別のターミナルでも、ROSと `install/setup.bash` を読み込んでください。

## 3. 入力データを用意する

サービスは、受信しておいた最新の点群から指定画素を読みます。
元教材の `person_static` または `person_dynamic` を配置し、再生してください。
bag本体はGitHubには含まれていません。

```bash
ros2 bag play data/rosbags/person_static --loop
```

元コードが購読するトピックは次のとおりです。

| トピック | 型 |
|---|---|
| `/camera/rgb/image_rect_color` | `sensor_msgs/msg/Image` |
| `/camera/depth_registered/image` | `sensor_msgs/msg/Image` |
| `/camera/depth_registered/points` | `sensor_msgs/msg/PointCloud2` |

3D位置の読み取りには、画像の行・列に対応した点群（organized cloud、`height > 1`）が必要です。
`video_*` と `hoodie_*` の保存済み点群は `height=1` なので、名前を付け替えるだけでは使えません。
`person_rgbd_1` もトピックと入力形式が異なります。

## 4. サービスを起動して呼び出す

別のターミナルでC++版のサービスを起動します。

```bash
ros2 run robovision_services robovision_service
```

点群を受信したログが出たら、さらに別のターミナルから確認します。

```bash
ros2 service list
ros2 service type /get_point_center
ros2 interface show robovision_interfaces/srv/GetPointCenter
ros2 service call /get_point_center robovision_interfaces/srv/GetPointCenter \
  "{x: 320, y: 240}"
```

応答の `point` に、列320・行240に対応するXYZが入ります。
640×480画像では中央付近ですが、座標は入力画像のサイズに合わせて指定してください。

### サービス側の流れ

1. 点群のコールバックで、最新データを保存する。
2. `/get_point_center` に要求が来たら、指定された列・行を確認する。
3. 対応する点のXYZを応答に入れて返す。

要求が来るまで画像の購読も止まるわけではありません。
また、この実装ではRGB・Depth・点群を同期していないため、同じ撮影時刻の組とは限りません。

## 5. クライアントから呼び出す

CLIでの手動呼び出しの代わりに、クライアントノードを使えます。
サービスとbagを起動したまま、別のターミナルで実行してください。

```bash
ros2 run robovision_services robovision_client --ros-args -p x:=320 -p y:=240
```

2.5秒ごとに指定座標を問い合わせます。`x` と `y` を変え、返る位置を比べてください。

クライアントでは、サービスが利用可能になるのを待ち、要求を送ってから応答を受け取ります。
Python版は `call_async` と応答時のコールバックを使い、C++版は別スレッドで応答を待ちます。
ROSのサービスは要求と応答の仕組みですが、呼び出し元を必ず同期的に待たせるものではありません。

### Python版

C++版のサービスとクライアントを止め、代わりに次をそれぞれ別のターミナルで実行します。

```bash
ros2 run robovision_services robovision_service.py
```

```bash
ros2 run robovision_services robovision_client.py --ros-args -p x:=320 -p y:=240
```

## 6. 教材コードの注意点

- 点群がまだ届いていないと、XYZは初期値の0になります。
  C++版では範囲外の要求も0を返します。成功・失敗を示すフィールドがないため、0だけで有効な位置とは判断できません。
- NaNなど、測定できなかった点の値が返ることもあります。
- Python版には `if request.x and request.y` という判定があり、列0・行0を扱えません。
  範囲外の要求にも注意が必要です。
- C++版の点群読み込みは、先頭にfloatのXYZが並ぶ特定のレイアウトを仮定しています。
  異なる点群に対応するには `fields` の確認やPointCloud2のイテレータが必要です。
- C++クライアントには、スレッドの終了処理や共有変数の排他制御を見直す余地があります。
  そのまま長時間運用するための実装ではありません。

これらは元教材の実装を読む際の注意点です。この日本語版の追加ではコード自体は変更していません。

## 7. 演習

1. 画像内の5か所を問い合わせ、XYZの変化を確認してください。
2. 点群の到着前、範囲外、NaNの点で、応答とログがどうなるか比べてください。
3. サービスの応答に成功・失敗と理由を追加するなら、どのような型にしますか？
4. 1画素ではなく小さな領域の有効点を集め、平均や中央値を返す処理を考えてください。
5. クライアントの問い合わせを1回だけに変える場合、タイマーと終了処理をどう変えますか？

## 参考

- [ROS 2 Humble：C++のService / Client](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
- [ROS 2 Humble：PythonのService / Client](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)

元教材の著者：**Luis Contreras**（[ARTenshi](https://artenshi.github.io/)）、
**Hiroyuki Okada**（[AIBot](http://aibot.jp/)）。日本語版はこのforkのコードに合わせて整理しています。
