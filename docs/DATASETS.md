# データの構成と注意点

## GitHubに含めるもの／含めないもの

GitHubにはコード、launch、RViz設定、ドキュメント、`metadata.yaml` を保存しています。
撮影動画、bag本体、モデルの重み、ビルド結果、認証情報は含めていません。
**メタデータだけでは再生できないので、clone後にデータを配置するか、動画から作成してください。**

元教材の `person_static` / `person_dynamic` の入手先は
[元README](UPSTREAM_README.md) を参照してください。
`person_rgbd_1` は手元で追加した実測RGB-Dデータです。このリポジトリでは配布していません。

## カスタムデータ

```text
data/rosbags/
├── video_01/ ... video_04/
└── hoodie_01/ ... hoodie_04/
    ├── data.db3
    ├── metadata.yaml
    └── video.mp4
```

8本とも解像度は1280×720で、元の30fpsから1フレームおきに取り出して15fpsにしています。
RGB、推定Depth、CameraInfo、色付き点群には、同じフレームの時刻を付けて保存しています。
点群は3画素おきに生成し、1フレームあたり102,480点です。RVizでは点を5pxで表示します。
8本の合計容量は約6GiBです。

| トピック | 内容 |
|---|---|
| `/camera/rgb/image/compressed` | JPEG RGB |
| `/camera/depth/image/compressed` | 16UC1 PNG、mm表現、2cm単位に量子化 |
| `/camera/rgb/camera_info` | 仮想カメラ情報（水平画角70度） |
| `/camera/depth/points` | XYZRGB PointCloud2、非organized（height=1） |

Depthは**表示用の疑似距離**です。モデルが出す相対的な逆Depthを、
各フレームの2～98パーセンタイルを基準に正規化し、0.5～5mに割り当てています。
実際の距離とは限らず、フレーム間で値が揺れることもあります。カメラの画角も仮定値です。
また、保存するDepthは2cm単位に丸めていますが、点群には丸める前の値を使うため、両者に小さな差があります。

動画ファイルから作る場合、bagの開始時刻にはファイルの更新時刻を使います。
撮影時刻とは限りません。各フレームの時刻は、元動画のfpsに合わせて付けます。

詳細な再生例は [VIDEO_BAGS.md](../data/rosbags/VIDEO_BAGS.md) を参照してください。
