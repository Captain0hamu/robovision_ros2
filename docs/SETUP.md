# セットアップと再現

## 基本環境

- Ubuntu 22.04 / ROS 2 Humble
- OpenCV / cv_bridge / image_transport / message_filters / RViz2
- 生成時のみ: Python 3、PyTorch、Transformers、NumPy、PyYAML

既存のDockerイメージ `tidbots/hsr-ros2:humble-nvidia` でも動作確認しています。
このforkには親ディレクトリのDocker設定・ネットワーク設定・パスワードは含めません。

```bash
git clone https://github.com/Captain0hamu/robovision_ros2.git
cd robovision_ros2
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/setup.bash
```

ROSをインストール済みで、rosdep初期化・必要なapt依存の導入権限があることが前提です。
GUIをコンテナで使う場合はDISPLAY/X11を別途設定してください。

## 推定モデルのPython依存

推論を行わないbag再生にはPyTorch・Transformersは不要です。
生成用には `tools/requirements-depth.txt` を参照してください。
PyTorchはCPU/CUDAに合うビルドを公式手順で導入します。
ROSのPythonパッケージが見えるPython環境で利用してください。

モデル: `LiheYoung/depth-anything-small-hf`。
初回はネット接続が必要で、約95MiBの重みがHugging Faceキャッシュへ保存されます。
検証時の組合せはPyTorch 2.12.1+cu130、Transformers 4.40.0、
NumPy 1.26.4、OpenCV 4.11.0です。すべてのバージョンでの動作を保証するものではありません。

## 動画を一度だけ処理して保存する

ROS環境・モデル依存を読み込んだ状態で実行します。

```bash
python3 tools/video_to_depth_bag.py /path/to/video_01.mp4 \
  /path/to/hoodie_01.mp4 --output-root data/rosbags
```

既存の出力は上書きしません。再生成時は別の出力先を使い、検証後に入れ替えてください。
GPU未対応環境ではCPUへフォールバックします。`--cpu` でCPUを明示できます。

Dockerで実行する例（既存イメージの利用）:

```bash
docker run --rm --gpus all --entrypoint bash \
  -v "$PWD:/workspace" \
  -v "$HOME/Videos/Camera/mp4:/videos:ro" \
  -v "$HOME/.cache/huggingface:/home/roboworks/.cache/huggingface" \
  tidbots/hsr-ros2:humble-nvidia -lc \
  'source /opt/ros/humble/setup.bash && cd /workspace && \
   python3 tools/video_to_depth_bag.py /videos/*.mp4 --output-root data/output_bags'
```

この例のキャッシュ先は検証イメージのユーザー設定用です。別イメージではHF_HOME等で調整します。
GPUを渡さない場合は `--gpus all` を外してください。

## 実測RGB-Dとの使い分け

- `person_static` / `person_dynamic`: `robovision_rgbd rgbd_reader_sync` を使用。
- `person_rgbd_1`: 圧縮RGB・compressedDepthを `rgbd_cloud_demo` で展開して点群化。
- `video_*` / `hoodie_*`: 点群保存済みなのでbagを再生するだけ。

トピック名・圧縮方式が異なるため、同じlaunchで混用しないでください。
同時起動するノードはROS_DOMAIN_ID・RMW_IMPLEMENTATIONを揃えます。
