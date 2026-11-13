# Lesson 2：OpenCVで画像を処理する

[English](README.md) · [前へ：画像の送受信](../robovision_1/README_ja.md) · [次へ：RGB-Dと点群](../robovision_3/README_ja.md)

ROSで受信した画像を使い、画素の読み取り、文字の描画、グレースケール変換、色の抽出を試します。
このフォルダのROSパッケージ名は `robovision_processing` です。

## 1. 起動する

以下はリポジトリのルートから実行します。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robovision_images robovision_processing
source install/setup.bash
ros2 run robovision_images my_publisher "$PWD/data/images/baboon.png"
```

別のターミナルで環境を読み込み、処理ノードを起動します。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robovision_processing my_processing
```

Python版を使う場合は、C++版の代わりにこちらを起動します。

```bash
ros2 run robovision_processing my_processing.py
```

どちらも `/camera/image` を受信します。カメラ映像で試すなら、静止画の配信を止めて
Lesson 1の `my_camera_publisher` を使ってください。表示にはGUIが必要です。

> C++版には、このforkで試したグレースケール変換、180度回転、色マスク、番号表示が入っています。
> Python版は元教材の基本形です。以下の処理例がすべて実装済みというわけではありません。
> 追加・変更したら、再ビルドしてから起動してください。

## 2. 画像の座標と色

画像は画素を縦横に並べた配列です。左上が原点で、右に進むと列、下に進むと行が増えます。

![画像の座標](images/digital_image.jpg)

OpenCVで扱う一般的な8bitカラー画像は、1画素がB・G・Rの3成分で、各値は0〜255です。
たとえばBGRの `[0, 0, 255]` は赤です。

| 調べるもの | C++ | Python |
|---|---|---|
| 高さ | `image.rows` | `image.shape[0]` |
| 幅 | `image.cols` | `image.shape[1]` |
| チャンネル数 | `image.channels()` | カラーなら `image.shape[2]` |
| 画素 | `image.at<cv::Vec3b>(row, col)` | `image[row, col]` |

グレースケール画像は1チャンネルです。Pythonでは通常、`shape` は `(高さ, 幅)` になります。

**配列は「行、列」、`cv::Point` は「x、y」、つまり「列、行」の順です。**
どちらも最後の有効な添字は「サイズ−1」です。

## 3. 文字を描く

```cpp
cv::putText(image_, std::to_string(counter_), cv::Point(25, 25),
            cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
```

```python
cv2.putText(self.img, str(self.counter), (25, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
```

位置、文字の大きさ、色、線の太さを変えてみてください。
`cv::Scalar` とPythonの色タプルは、ここではBGR順です。
C++の `CV_RGB(r, g, b)` はRGB順の引数をBGRへ並べ替えるマクロなので、混同しないようにします。

![文字の表示例](images/baboon_number.jpg)

## 4. グレースケールに変換する

```cpp
cv::Mat gray;
cv::cvtColor(image_, gray, cv::COLOR_BGR2GRAY);
cv::imshow("gray", gray);
```

```python
gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
cv2.imshow("gray", gray)
```

明るさは、概ね次の重み付けで計算できます。

```text
Gray ≈ 0.30 × R + 0.59 × G + 0.11 × B
```

これは学習用の近似式です。OpenCVの変換とは係数や丸め方による差が出ることがあります。
変換後も幅と高さは同じですが、チャンネル数は3から1になります。

![グレースケールの例](images/baboon_gray.jpg)

## 5. 色の範囲で抽出する

`inRange` は、各成分が指定範囲に収まる画素を選びます。
次の例ではBが0〜240、Gが0〜255、Rが220〜240の画素を選びます。
「赤い物体だけ」を必ず抽出できる条件ではないので、画像を見ながら範囲を調整してください。

```cpp
cv::Mat mask, filtered;
cv::inRange(image_, cv::Scalar(0, 0, 220), cv::Scalar(240, 255, 240), mask);
image_.copyTo(filtered, mask);
cv::imshow("mask", mask);
cv::imshow("filtered", filtered);
```

Pythonで青の成分を基準に選ぶ例です。

```python
import numpy as np

lower = np.array([200, 0, 0], dtype=np.uint8)
upper = np.array([240, 255, 255], dtype=np.uint8)
mask = cv2.inRange(self.img, lower, upper)
filtered = cv2.bitwise_and(self.img, self.img, mask=mask)
```

マスクは1チャンネルで、選ばれた画素が255、その他が0です。
`copyTo` や `bitwise_and` で、選んだ部分だけを元画像から取り出せます。
照明の変化に対応したい場合は、HSV色空間での抽出も試してみてください。

![赤成分を使った抽出例](images/baboon_mask_red.jpg)

## 6. 画素を直接読む

画像中央のBGR値を読む例です。

```cpp
const int row = image_.rows / 2;
const int col = image_.cols / 2;
const cv::Vec3b pixel = image_.at<cv::Vec3b>(row, col);
```

```python
height, width = self.img.shape[:2]
pixel = self.img[height // 2, width // 2]
```

グレースケール画像なら、C++では `gray.at<uchar>(row, col)` を使います。
数値として出力するときは、`uchar` を `int` に変換すると文字扱いを避けられます。

## 7. 全画素を処理する

グレースケール変換を自分で書くと、配列の読み書きを練習できます。

```cpp
cv::Mat gray(image_.rows, image_.cols, CV_8UC1);
for (int row = 0; row < image_.rows; ++row) {
    for (int col = 0; col < image_.cols; ++col) {
        const auto pixel = image_.at<cv::Vec3b>(row, col);
        gray.at<uchar>(row, col) = static_cast<uchar>(
            0.11 * pixel[0] + 0.59 * pixel[1] + 0.30 * pixel[2]);
    }
}
```

```python
height, width = self.img.shape[:2]
gray = np.zeros((height, width), dtype=np.uint8)
for row in range(height):
    for col in range(width):
        b, g, r = (int(value) for value in self.img[row, col])
        gray[row, col] = int(0.11 * b + 0.59 * g + 0.30 * r)
```

Pythonでの画素ごとのループは遅くなりやすいため、通常は `cvtColor` などのOpenCV関数や
NumPyの配列演算を使います。ここでは計算の仕組みを確認するためにループを書いています。

## 8. 演習

1. 元画像とグレースケール画像の幅、高さ、チャンネル数を確認してください。
2. 文字の色と位置を変え、5種類の表示を作ってください。
3. 色の抽出範囲を変え、静止画とカメラ映像で結果を比べてください。
4. 10か所の画素についてBGR値を読み、近似式で求めた明るさと `cvtColor` の結果を比べてください。
5. 別の画像を用意し、画素をコピーするループで180度回転してください。
   元の `(行, 列)` と、回転後の `(高さ−1−行, 幅−1−列)` の対応がヒントです。

![180度回転の例](images/baboon_rotated.jpg)

このforkのC++版では、5の回転とグレースケール変換を同じループで行っています。
元画像と同じ向きの変換結果と比較すると、添字の働きを確認できます。

元教材の著者：**Luis Contreras**（[ARTenshi](https://artenshi.github.io/)）、
**Hiroyuki Okada**（[AIBot](http://aibot.jp/)）。日本語版はこのforkのコードに合わせて整理しています。
