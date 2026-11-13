# データ管理

## GitHubに含めるもの／含めないもの

含める: コード、launch、RViz設定、説明、`metadata.yaml`。
含めない: 個人撮影動画、`.db3`等のbag実体、モデル重み、ビルド成果物、認証情報。
**メタデータだけでは再生できません。clone後は実データの配置または再生成が必要です。**
個人撮影データをRelease等へ自動アップロードすることも行いません。

元教材の `person_static` / `person_dynamic` の入手先は
[元README](UPSTREAM_README.md) を参照してください。
`person_rgbd_1` はローカルで追加された実測RGB-Dデータで、本forkは実体を配布しません。

## カスタムデータ

```text
data/rosbags/
├── video_01/ ... video_04/
└── hoodie_01/ ... hoodie_04/
    ├── data.db3
    ├── metadata.yaml
    └── video.mp4
```

全8本は1280×720、元30fpsから15fpsへ間引き。
RGB・推定Depth・CameraInfo・色付き点群を同じtimestampで保存します。
点間隔3画素で102,480点/フレーム、RViz表示サイズは5pxです。
ローカルで生成した8本の総容量は約6GiBです。

| トピック | 内容 |
|---|---|
| `/camera/rgb/image/compressed` | JPEG RGB |
| `/camera/depth/image/compressed` | 16UC1 PNG、mm表現、2cm単位に量子化 |
| `/camera/rgb/camera_info` | 仮想カメラ情報（水平画角70度） |
| `/camera/depth/points` | XYZRGB PointCloud2、非organized（height=1） |

Depthは各フレームの相対逆Depthを百分位2～98%で正規化し、
表示用の0.5～5mへ割り当てた**疑似距離**です。
真の尺度・カメラ校正・時間的一貫性は保証されません。
保存Depthは量子化済み、保存点群は量子化前の推定値から作るため、小さな差があります。

ファイル入力の場合、bagの開始時刻は元動画の更新時刻を使用します。
撮影時刻であるとは限りません。フレーム間隔は元動画のfpsから生成します。

詳細な再生例は [VIDEO_BAGS.md](../data/rosbags/VIDEO_BAGS.md) を参照してください。
