# Robot Vision — ROS 2 カスタム実験集

[ARTenshi/robovision_ros2](https://github.com/ARTenshi/robovision_ros2) の教材をベースに、
RGB-Dの同期や色付き点群の生成、通常のカメラ動画からのDepth推定を追加しています。
推定結果はROS bagに保存できるため、再生のたびに推論する必要はありません。
元教材の著者は Luis Contreras / Hiroyuki Okada です。

> **動画から作成したDepth・点群はAIによる推定です。実測距離ではありません。**
> 距離計測・衝突回避・安全制御には使用しないでください。

## ドキュメント

| 内容 | ドキュメント |
|---|---|
| 追加した機能と動作確認の結果 | [カスタマイズレポート](docs/CUSTOMIZATION_REPORT.md) |
| 環境構築とデータの作り方 | [セットアップ](docs/SETUP.md) |
| データの構成と注意点 | [データガイド](docs/DATASETS.md) |
| 動画から推定Depth・点群を生成 | [単眼Depth](robovision_1/MONOCULAR_DEPTH.md) |
| RGB-D同期・顔検出・点群生成 | [RGB-D同期](robovision_3/RGBD_SYNC.md) |
| 圧縮RGB-Dから点群を生成 | [rgbd_cloud_demo](test/README.md) |
| 元教材 | [オリジナルREADME](docs/UPSTREAM_README.md) |

### レッスンの日本語版

1. [画像を送受信する](robovision_1/README_ja.md)
2. [OpenCVで画像を処理する](robovision_2/README_ja.md)
3. [RGB-D画像と点群を扱う](robovision_3/README_ja.md)
4. [サービスとクライアントを使う](robovision_4/README_ja.md)

## bagを再生する

Ubuntu 22.04 / ROS 2 Humble 環境で、リポジトリ直下から実行します。
**bagと動画はGitHubには含めていません。手元のデータを配置するか、動画から作成してください。**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robovision_images
source install/setup.bash

# Publishのみ。保存済み点群なのでAI推論不要
ros2 bag play data/rosbags/video_01 --loop
```

bagとRVizをまとめて起動する場合はこちらを使います。上の再生コマンドとは、どちらか一方を実行してください。

```bash
ros2 launch robovision_images play_depth_bag.launch.py \
  bag:=$PWD/data/rosbags/hoodie_01
```

RVizの描画にGPUを使うことはありますが、bagの再生中にDepthの推論は行いません。

## ディレクトリ構成

```text
robovision_1/           画像教材・単眼Depthノード・再生launch・RViz設定
robovision_2/           グレースケール・色マスク等の画像処理教材
robovision_3/           実測RGB-Dの同期・顔検出・点群生成
robovision_4/           ROSサービス教材
robovision_interfaces/ メッセージ・サービス定義
test/                  独立ROSパッケージ rgbd_cloud_demo
tools/                 動画→bagのオフライン変換
docs/                  セットアップ・データ説明・変更レポート
data/                  教材画像・bagメタデータ（動画やbag本体は手元で管理）
```

`rgbd_reader.cpp` と `tmp.cpp` は過去の実験コードとして残しています。
これから使う場合は、動作確認済みの `rgbd_reader_sync` または `rgbd_cloud_demo` を選んでください。

## 権利・公開範囲

元教材の著者表記はそのまま残しています。元リポジトリには全体のLICENSEが見当たらないため、
このforkでも新たなライセンスは付けていません。
モデル、OpenCV付属のCascade、外部データの利用条件は、それぞれの配布元を確認してください。
個人で撮影した動画やbag、モデルの重み、認証情報は公開していません。
