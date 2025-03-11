# Lab-pointcloud-tool
研究で使用した点群と画像処理のtool  


自分用NOTE: 使用したやつ旧バージョンも含めてるので、注意してください。
PCLをなぜ使わなかったのか後悔。  


## C++ 環境設定
- opencv install  
libopencv-dev cmake

- eigen install  
[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)のget itからDLし、 展開した中のEigenディレクトリを/usr/local/include/eigen3にコピーする。

- rapidjson install
jsonの解析に使用
Eigenと同じheader onlyなファイルなので、[rapidjson](https://github.com/Tencent/rapidjson)からcloneし、/usr/local/include/rapidjson/に rapidjson/include/rapidjson配下を移動する。


- open3d install (optional)  
[open3d](http://www.open3d.org/docs/release/getting_started.html)のinstallを参考にインストール  
自分でbuildしないと入らない。
meshlabで結果を確認するのが面倒なときに使用した。


- matplotlib-cpp install (optional)  
計算結果の可視化に使用  
ヘッダーを/usr/local/includeに配置する  



## build
build directoryを作成して、その中でcmake, makeを実行  

```bash
cmake ..
make
```

debug, open3d, matplotlibのflagを有効にする場合
```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug -DOPEN3D_ENABLED=ON -DMATPLOTLIB_ENABLED=ON
```

開発用
```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug -DOPEN3D_ENABLED=ON -DMATPLOTLIB_ENABLED=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## 実行
optionのmodeで機能の切り替えを行う。shellscriptなどで実行すると楽。

- mode1: 回転と並進を対応点から計算し、求めた回転並進を点群に適応して出力
- mode2: 対応点から回転のみの計算、点群に適応して出力
- mode3: *OLD* bbox内の点群を抽出、クラスごとに出力
- mode4: *OLD* 画像のshiftによって、類似度を計算し、対応を得る
- mode5: stripe_patternを使用した画像のshiftのテスト
- mode6: bbox内の点群に抽出、クラスごとに出力
- mode7: 画像と点群の最も類似度が高い回転角を探索
- mode8: 画像と点群の最も類似度が高い高さを探索
- mode9: segmentation結果を読み込んで、対応する点群をクラスごとに出力


### 画像と点群対応から回転並進を計算
回転と並進を対応点から計算する  
in  
- 360°画像
- plyファイル
- 画像対応点
- plyの対応点

out  
- 画像の座標に位置合わせしたplyファイル

```sh
./build/Rotation \
--ply ply_file_path \
--img img_file_path \
--ply_cp plycp_path \
--img_cp imgcp_path \
--mode 1
```

対応点形式  
最初の行に点の数を記述し、そのあとに点座標  
点数が同じである必要あり。


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


### bbox, segmentation結果から点群を抽出
物体検出によって取得したbbox, pixelのjsonファイル, 位置合わせ済みの点群と画像を読み込んでbbox内の点群を取得する

in  
- 360°画像
- plyファイル
- detectしたjsonファイル

out  
- 抽出したクラスごとの点群

```sh
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



### 画像と点群の対応を取得
**z軸方向と高さの2自由度に固定してることを忘れず**
画像と点群のエッジ画像から類似度を計算し、最も類似度が高い姿勢を探索する。

in
- 全方位画像
- 計測したplyファイル

out
- 対応点

```sh
./build/Rotation \
--ply ply_file_path \
--img imt_file_path \
--dir out_dir_path \
--mode 7 
```
