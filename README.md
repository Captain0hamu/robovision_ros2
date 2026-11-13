# Robot Vision — ROS 2 カスタム実験集

[ARTenshi/robovision_ros2](https://github.com/ARTenshi/robovision_ros2) の教材をベースに、
RGB-D同期、色付き点群生成、単眼動画のDepth推定、オフラインbag生成を追加したforkです。
元教材の著者は Luis Contreras / Hiroyuki Okada です。

> **動画から作成したDepth・点群はAIによる推定です。実測距離ではありません。**
> 距離計測・衝突回避・安全制御には使用しないでください。

## ドキュメント

| 内容 | ドキュメント |
|---|---|
| 変更内容・設計・検証・制限のレポート | [カスタマイズレポート](docs/CUSTOMIZATION_REPORT.md) |
| 環境構築・再現方法 | [セットアップ](docs/SETUP.md) |
| データの管理・公開方針 | [データガイド](docs/DATASETS.md) |
| 動画から推定Depth・点群を生成 | [単眼Depth](robovision_1/MONOCULAR_DEPTH.md) |
| RGB-D同期・顔検出・点群生成 | [RGB-D同期](robovision_3/RGBD_SYNC.md) |
| 圧縮RGB-Dから点群を生成 | [rgbd_cloud_demo](test/README.md) |
| 元教材 | [オリジナルREADME](docs/UPSTREAM_README.md) |

## 最短で再生する

Ubuntu 22.04 / ROS 2 Humble 環境で、リポジトリ直下から実行します。
**bag・動画は公開リポジトリに含まれません。別途配置または生成してください。**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robovision_images
source install/setup.bash

# Publishのみ。保存済み点群なのでAI推論不要
ros2 bag play data/rosbags/video_01 --loop
```

またはbag再生とRVizをまとめて起動します（上記のbag再生とは同時起動しない）。

```bash
ros2 launch robovision_images play_depth_bag.launch.py \
  bag:=$PWD/data/rosbags/hoodie_01
```

RVizの描画にはGPUを使う場合がありますが、保存済みbagの再生ではDepth推論しません。

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
data/                  教材画像・bagメタデータ（大容量実体はローカル）
```

`rgbd_reader.cpp` と `tmp.cpp` は既存の実験コードとして保持しています。
新規利用では検証済みの `rgbd_reader_sync` または `rgbd_cloud_demo` を推奨します。

## 権利・公開範囲

元教材の著者表記を保持しています。上流にリポジトリ全体のLICENSEが確認できないため、
本forkで独自に全体へライセンスを付け直していません。
モデル・OpenCV付属のCascade・外部データの利用条件は各配布元を確認してください。
個人撮影動画・bag・モデル重み・認証情報は公開対象外です。
