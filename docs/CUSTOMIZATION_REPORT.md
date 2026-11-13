# カスタマイズレポート

## 1. ベースと移行範囲

上流: [ARTenshi/robovision_ros2](https://github.com/ARTenshi/robovision_ros2)。
既存の非公開作業repo `Robovision_env` に保存されていた教材実験と、
ローカルで追加したRGB-D・単眼Depth関連の変更をこのforkへ統合しました。
上流の履歴・著者表記・教材説明は保持します。

## 2. 既存の教材カスタム

- Lesson 1: 原寸／半分サイズ画像のPublishとSubscribe。
- Lesson 2: グレースケール、反転画像、色マスク、フレーム番号表示等。
- Lesson 3: 日本語解説、RGB-D同期の試作、サンプル点のPose出力。

`rgbd_reader.cpp` は実験途中のコードを保持しており、CameraInfoトピックを
PointCloud2として購読する等の問題が残っています。推奨経路では使用しません。
`src/tmp.cpp` は未ビルドの試作です。削除済みだった `src/comment.cpp` は復元せず、
旧コミットから参照できます。未確認コードの一括削除・書換えは行っていません。

## 3. 実測RGB-Dの同期処理

`robovision_3/src/rgbd_reader_sync.cpp` はRGB・Depth・RGB CameraInfoを
ApproximateTimeで同期し、顔検出表示と点群生成を同じcallbackで処理します。
Depthの16UC1はmm→m、32FC1はmとして扱い、校正行列からXYZを計算します。
RGBに位置合わせ済みのDepthを前提とし、位置合わせ自体は行いません。

独立パッケージ `test/`（`rgbd_cloud_demo`）は、`person_rgbd_1` の
圧縮RGB・compressedDepthをノード内で展開します。
両経路のトピック・制限・詳細検証は各READMEへ分離しています。

## 4. 単眼動画からの推定データ作成

通常のUbuntuカメラ動画には実測Depthがないため、Depth Anything Smallで
相対Depthを推定します。リアルタイム版も残していますが、日常利用はオフライン版です。

```text
MP4 → Depth Anything（初回生成時のみ）
    → 同期RGB / 推定Depth / 仮想CameraInfo / XYZRGB
    → data.db3 + metadata.yaml
    → ros2 bag play（推論なし）
```

GPU発熱を抑えるため、毎回推論する方式から結果保存方式へ変更しました。
点群密度は12→6→3画素間隔、保存レートは10→15fpsへ改善。
最終条件は102,480点/フレーム、15fps、点表示5pxです。
Camera動画4本とHoodie動画4本を同じ条件で用意しました。
元動画が30fpsなので、60fps化や偽フレームの補間は行っていません。

## 5. 検証とその範囲

ローカルDocker / ROS 2 Humble環境で以下を確認しています。

- `robovision_images` のビルド、launch引数の解決。
- 移行時に独立したビルド領域で `robovision_images`、`robovision_rgbd`、
  `rgbd_cloud_demo` のクリーンビルドを再確認。
  旧 `rgbd_reader.cpp` の未使用変数 `roll` に警告が1件残る。
- 生成済み全8bagのメタデータ・画像復号・102,480点・同期timestamp。
- コピーした元MP4のSHA-256一致。
- GPUを渡さないbag再生でPointCloudを受信。
- `hoodie_01` の受信レートが平均15.00Hz。
- 以前の実測RGB-D検証: `person_rgbd_1` の点群生成、static/dynamicの同期点群生成。

画像・点群データの読出しと通信確認は、すべての視点での見た目の保証ではありません。
RViz描画は環境によりGPUを使い、描画負荷やDDS転送量は密度に伴い増えます。
「GPU推論なし」は「GPU使用率が必ずゼロ」という意味ではありません。

## 6. 限界・今後の課題

- 単眼推定の絶対距離は不明。疑似m値を実測値として扱わない。
- フレームごとの正規化によりDepthが時間的に揺れる可能性がある。
- CameraInfoは仮定画角であり、実カメラを校正した値ではない。
- 点群は表面をサンプリングした点であり、メッシュや隠れた面の復元ではない。
- 公開repoには約6GiBのデータ実体を含めない。再現には入力動画が必要。
- 上流全体のライセンスが不明なため、新たな全体ライセンスを付与しない。
