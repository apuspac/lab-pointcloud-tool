#!/bin/bash


# 空間内に，異なる（同じ平面上に載らない）平面格子を2枚置く．
    # 2つのgridを作る。
# 水平方向に少し「引いた」位置（カメラ位置）から撮影したデータを作る。
    # 並進を引いたデータを作る(投影処理は結局プログラムで行うのでヨシ)
# そこから，更に少しずれた位置に移動し（LiDAR位置），データを作成する．
    # カメラ位置のデータに並進回転を加える。

generate_dir="out/grid-t300-r001d30-3"

rm -rf $generate_dir

mkdir -p $generate_dir

function check()
{
    local program_name=`basename "$1"`
    echo run: $program_name >&2
    if ! $*; then
        echo error: $program_name >&2
        exit 1
    fi
}


# grid作成
./make_grid/make_grid -W 10 -H 10 -n 2 -m 2 > ./$generate_dir/grid1.dat
./make_grid/make_grid -W 8 -H 8 -n 2 -m 2 > ./$generate_dir/grid2.dat

# grid 2つを作成して、merge
#grid1
# 1: 8,10,0
# 2: -12,10,0
# 3: 12, 10, 0

./rotate/rotate -a 1,0,0 -d -30 > ./$generate_dir/grid11.dat < ./$generate_dir/grid1.dat
./rotate/rotate -a 0,1,0 -d 90 > ./$generate_dir/grid111.dat < ./$generate_dir/grid11.dat
./translate/translate -t 12,12,15 > ./$generate_dir/grid1-tr.dat < ./$generate_dir/grid111.dat

# grid2
# 1: 3,15,0
# 2: -17,15,0
# 3: 17,15,0

./rotate/rotate -a 1,0,0 -d -30 > ./$generate_dir/grid22.dat < ./$generate_dir/grid2.dat
./rotate/rotate -a 0,1,0 -d 90 > ./$generate_dir/grid222.dat < ./$generate_dir/grid22.dat
./translate/translate -t 14,12,15 > ./$generate_dir/grid2-tr.dat < ./$generate_dir/grid222.dat

./merge/merge ./$generate_dir/grid1-tr.dat ./$generate_dir/grid2-tr.dat > ./$generate_dir/scene.dat

# 画像データを作成
# ちょっと引いたところからのデータ(並進を引く)を作成する。
./translate/translate -t 5.0,5.0,0  < ./$generate_dir/scene.dat > ./$generate_dir/img.dat

# LiDARデータ作成
# 画像データに並進回転を加える。
# imgの位置 (-5, -5, 0)元データ基準
# からxが3m程度離れた (3.0, 0, 0) 回転軸(0,0,1.0) 回転角度30度
# 計測したとする
# t(3.0, 0, 0) Ra(0, 0, 1.0) Rd(30)
# t(3.0, -1.0, 0) Ra(0, 0, 1.0) Rd(30)
# t(3.0, -5.0, 0) Ra(0.5, 0.5, 1.0) Rd(50)


./translate/translate -t 3.0,0,0 > ./$generate_dir/lidar1.dat < ./$generate_dir/img.dat
./rotate/rotate -a 0,0,1.0 -d 30 > ./$generate_dir/lidar.dat < ./$generate_dir/lidar1.dat



# ply出力
./3d_to_ply/3d_to_ply < ./$generate_dir/scene.dat \
    -o ./$generate_dir/scene.ply

./3d_to_ply/3d_to_ply < ./$generate_dir/img.dat \
    -o ./$generate_dir/img.ply

./3d_to_ply/3d_to_ply < ./$generate_dir/lidar.dat \
    -o ./$generate_dir/lidar.ply
