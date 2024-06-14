# coordinate_transform
Sumitomo関連のプログラムをまとめたもの


## 環境設定
- opencv install  
libopencv-dev cmake

- eigenインストール  
[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)のget itからDLし、 展開した中のEigenディレクトリを/usr/local/include/eigen3にコピーする。

- open3d install (optional)
[open3d](http://www.open3d.org/docs/release/getting_started.html)のinstallを参考にインストール  
自分でbuildしないと入らない。

- matplotlib-cpp install (optional)
ヘッダーを/usr/local/includeに配置する  



## build
build directoryを作成して、その中でcmake, makeを実行  

```bash
cmake ..
make
```

open3d, matplotlibを使う場合 flagを有効に
```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug -DOPEN3D_ENABLED=ON -DMATPLOTLIB_ENABLED=ON
```

開発用
```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug -DOPEN3D_ENABLED=ON -DMATPLOTLIB_ENABLED=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## 実行
modeで機能の切り替えを行う。
3つ機能(を実装するつもり).


### transform
回転と並進を対応点から計算する  
in  
- 360°画像
- plyファイル
- 画像対応点
- plyの対応点

out  
- 画像の座標に位置合わせしたplyファイル

```
./build/Rotation \
--ply ply_file_path \
--img img_file_path \
--ply_cp plycp_path \
--img_cp imgcp_path \
--mode 1
```

対応点形式  
最初の行に点の数を記述し、そのあとに点座標  
点数が同じである必要があります。



画像
```
8
3362 1581
3549 1587
3364 1966
3557 1961
806 1672
866 1695
806 1952
865 1952
```
点群
```
8
-0.151437 -4.928407 -0.302440
-0.175075 -4.988512 1.183640
-1.719230 -5.165187 1.178979
-1.653616 -5.008869 -0.234914
-1.720427 -1.819300 -1.207758
-1.723518 -1.366534 -1.209202
-1.708083 -0.932050 -1.207408
-1.697996 -0.438499 -1.208895
```


### capture_point_inner_bbox
物体検出によって取得したbbox, pixelのjsonファイル, 位置合わせ済みの点群と画像を読み込んでbbox内の点群を取得する

in  
- 360°画像
- plyファイル
- detectしたjsonファイル

out  
- 抽出したクラスごとの点群

```
./build/Rotation \
--ply ply_file_path \
--img img_file_path \
--json path/json_file \
--mode 0
```

読み込むdetectファイルの形式例: 
```json
{
    "input_path": "data/path",
    "merged_data": [
        {
            "file_path": "data/path/PIC.jpg",
            "file_name": "PIC.jpg",
            "bbox_info": [
                {
                    "xmin": 527.2387695312,
                    "ymin": 442.8101196289,
                    "xmax": 632.9676513672,
                    "ymax": 51760.787109377,
                    "confidence": 0.7604297996,
                    "class": 9,
                    "name": "square_box_w_marker_front_side"
                },
 
            ],
            "mask_info": [
                {
                    "class_name": "pipe",
                    "contour": [
                        [
                            [
                                889,
                                578
                            ]
                        ],
                        [
                            [
                                888,
                                579
                            ]
                        ],
                    ],
                    "confidence": 0.3661523759365082
                },
            ]
        }
    ]
}

```



### get_correspoing_point
画像と点群の対応を取って点として出力するのか 位置合わせを行うのか 未定.

in
- 全方位画像
- 計測したplyファイル

out
- 対応点



