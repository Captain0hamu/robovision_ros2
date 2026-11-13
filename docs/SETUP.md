# 環境構築とデータの作成

## 基本環境

- Ubuntu 22.04 / ROS 2 Humble
- OpenCV / cv_bridge / image_transport / message_filters / RViz2
- Depth推定を行う場合: Python 3、PyTorch、Transformers、NumPy、PyYAML

Dockerイメージ `tidbots/hsr-ros2:humble-nvidia` で動作確認しています。
手元のDocker設定やネットワーク設定は、このリポジトリには含めていません。

```bash
git clone https://github.com/Captain0hamu/robovision_ros2.git
cd robovision_ros2
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/setup.bash
```

先にROSのインストールとrosdepの初期化を済ませてください。
依存パッケージのインストールには管理者権限が必要です。
コンテナからGUIを表示する場合は、DISPLAY/X11も設定してください。

## Depth推定に必要なPythonパッケージ

bagを再生するだけなら、PyTorchとTransformersは不要です。
動画からDepthを推定する場合は、`tools/requirements-depth.txt` を参照してください。
PyTorchは公式手順に従い、使用するCPU/CUDAに合ったものをインストールします。
実行には、ROSのPythonパッケージも読み込める環境が必要です。

使用モデルは `LiheYoung/depth-anything-small-hf` です。
初回に約95MiBの重みをダウンロードし、Hugging Faceのキャッシュに保存します。
動作確認に使ったバージョンは、PyTorch 2.12.1+cu130、Transformers 4.40.0、
NumPy 1.26.4、OpenCV 4.11.0です。

## 動画を一度だけ処理して保存する

ROS環境を読み込み、上記のPythonパッケージをインストールしてから実行します。

```bash
python3 tools/video_to_depth_bag.py /path/to/video_01.mp4 \
  /path/to/hoodie_01.mp4 --output-root data/rosbags
```

同名のbagがあると処理を止めるため、作り直すときは別の出力先を指定してください。
GPUが使えない場合は自動でCPUに切り替わります。`--cpu` を付ければ、GPUがあってもCPUで処理します。

Dockerで実行する場合：

```bash
docker run --rm --gpus all --entrypoint bash \
  -v "$PWD:/workspace" \
  -v "$HOME/Videos/Camera/mp4:/videos:ro" \
  -v "$HOME/.cache/huggingface:/home/roboworks/.cache/huggingface" \
  tidbots/hsr-ros2:humble-nvidia -lc \
  'source /opt/ros/humble/setup.bash && cd /workspace && \
   python3 tools/video_to_depth_bag.py /videos/*.mp4 --output-root data/output_bags'
```

キャッシュの保存先は、このイメージのユーザー名に合わせています。
別のイメージでは、マウント先や `HF_HOME` を調整してください。
GPUを使わない場合は `--gpus all` を外します。

## 実測RGB-Dとの使い分け

- `person_static` / `person_dynamic`: `robovision_rgbd rgbd_reader_sync` を使用。
- `person_rgbd_1`: 圧縮RGB・compressedDepthを `rgbd_cloud_demo` で展開して点群化。
- `video_*` / `hoodie_*`: 点群保存済みなのでbagを再生するだけ。

データごとにトピック名や圧縮方式が違うため、対応するlaunchを使ってください。
通信するノード同士で、ROS_DOMAIN_IDとRMW_IMPLEMENTATIONを揃える必要があります。
