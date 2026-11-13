# カスタマイズレポート

## 1. 元にした教材

[ARTenshi/robovision_ros2](https://github.com/ARTenshi/robovision_ros2) をforkし、
以前の `Robovision_env` で行った実験と、手元で追加したRGB-D・単眼Depthの機能をまとめました。
元教材の履歴、著者表記、説明はそのまま残しています。

## 2. 教材に加えた変更

- Lesson 1：元画像と半分に縮小した画像の配信・受信。
- Lesson 2：グレースケール変換、画像の反転、色マスク、フレーム番号の表示。
- Lesson 3：日本語の解説、RGB-D同期の試作、画像内のサンプル点からPoseを出力する処理。

`rgbd_reader.cpp` は実験途中のまま残しています。CameraInfoのトピックを
PointCloud2として購読するなどの問題があるため、通常は後述の同期ノードを使ってください。
`src/tmp.cpp` も試作コードで、ビルドには含めていません。
削除済みの `src/comment.cpp` は過去のコミットから確認できます。

## 3. 実測RGB-Dの同期処理

`robovision_3/src/rgbd_reader_sync.cpp` はRGB・Depth・RGB CameraInfoを
ApproximateTimeで同期し、同じコールバックで顔検出の表示と点群の生成を行います。
Depthが16UC1ならmmからmに変換し、32FC1ならそのままmとして使います。
XYZはCameraInfoの校正行列から計算します。DepthはRGBに位置合わせされている必要があり、
このノードでは位置合わせを行いません。

`test/` にある別パッケージ `rgbd_cloud_demo` は、`person_rgbd_1` の
圧縮RGBとcompressedDepthをノード内で展開して点群を作ります。
トピックや使い方、動作確認の詳細は [RGB-D同期](../robovision_3/RGBD_SYNC.md) と
[rgbd_cloud_demo](../test/README.md) を参照してください。

## 4. 単眼動画からの推定データ作成

通常のカメラアプリで撮った動画にはDepthがないため、Depth Anything Smallで奥行きを推定します。
リアルタイムで推定するノードもありますが、普段は結果をbagに保存して使います。

```text
MP4 → Depth Anything（初回生成時のみ）
    → 同期RGB / 推定Depth / 仮想CameraInfo / XYZRGB
    → data.db3 + metadata.yaml
    → ros2 bag play（推論なし）
```

再生のたびに推論するとGPUが熱くなるため、一度作った結果を保存する方式に変えました。
点の間隔は12画素から6画素、さらに3画素へ狭め、保存するfpsも10から15に上げています。
現在は1フレームあたり102,480点、15fpsで、RVizの点サイズは5pxです。
Camera動画4本とHoodie動画4本を、この条件で作成しました。
元動画は30fpsです。60fps化や中間フレームの生成は行っていません。

## 5. 動作確認

手元のDocker / ROS 2 Humble環境で、次の項目を確認しました。

- `robovision_images`、`robovision_rgbd`、`rgbd_cloud_demo` が新しいビルド領域でビルドできること。
  旧 `rgbd_reader.cpp` には、未使用変数 `roll` の警告が1件残っています。
- `robovision_images` のlaunch引数を読み込めること。
- 全8bagの画像を復号でき、点群が102,480点で、4トピックの時刻が一致すること。
- コピーしたMP4と元ファイルのSHA-256が一致すること。
- GPUなしのbag再生でPointCloudを受信できること。
- `hoodie_01` を平均15.00Hzで受信できること。
- 実測RGB-Dから点群を作れること（`person_rgbd_1`、`person_static`、`person_dynamic`。以前の確認結果）。

ここで確認したのは、データを読み出して配信・受信できることです。
点群の隙間の見え方は、視点や表示設定によって変わります。
また、点数を増やすほど描画と通信は重くなります。推論をしなくても、RVizの描画にGPUを使うことはあります。

## 6. 注意点と残っている課題

- 単眼推定では実際の距離は分かりません。m表記でも、実測値としては使えません。
- フレームごとに正規化するため、Depthが時間とともに揺れることがあります。
- CameraInfoには仮の画角を使っており、実カメラを校正した値ではありません。
- 点群は見えている表面の点です。面を張ったメッシュや、隠れた部分は作っていません。
- 約6GiBのbagと動画はGitHubには含めていません。同じデータを作るには元動画が必要です。
- 元教材全体のライセンスが不明なため、このforkでも新たな全体ライセンスは付けていません。
