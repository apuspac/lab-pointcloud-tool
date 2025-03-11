#!/bin/bash

DIR="../res"
CALIB_DIR="../lab-pointcloud-tool"
DETECT_DIR="../lab-detect"


cd ../CALIB_DIR/

# z-axis rotate
./build/Rotation \
--ply $DIR/data/ply/Frame_03.ply \
--img $DIR/data/img/PIC_06181.jpg \
--dir $DIR/out_rotate/ \
--mode 7 

# height
./build/Rotation \
--ply $DIR/data/ply/Frame_03.ply \
--img $DIR/data/img/PIC_06181.jpg \
--dir $DIR/out_height/ \
--mode 8 


# detect_by_yolov5.pyなどのあるdetectディレクトリを準備
echo ">>>>>>>> Running YOLOv5"
cd ../$DETECT_DIR/
rye run python detect_by_yolov5.py ../$DIR/data/img ../$DIR/out/detect
#
echo ">>>>>>>> Running YOLACT"
cd ../$DETECT_DIR/
rye run python detect_by_yolact.py  ../$DIR/data/img ../$DIR/results/output_images
#
echo ">>>>>>>> Running merge_results.py"
rye run python merge_results.py ../$DIR/data/img ../$DIR/out/detect





echo ">>>>>>>> Running capturepoint"
# >>>>> bbox
cd ../$CALIB_DIR/
./build/Rotation \
--ply ../$DIR/out_height/mse_lidar_height.ply \
--img ../$DIR/data/img/PIC_06181.jpg \
--json ../$DIR/out_height/detect/detections.json \
--dir ../$DIR/out_parts/ \
--mode 6

# >>>>> mask
./build/Rotation \
--ply ../$DIR/out_height/mse_lidar_height.ply \
--img ../$DIR/data/img/PIC_06181.jpg \
--json ../$DIR/out_detect_mask/detections.json \
--dir ../$DIR/out_parts_mask/ \
--mode 9


# out_detect_maskにjsonファイルを入れて、結果もここに出力
# rye run python merge_results.py ../rememver_res/data/img ../rememver_res/out_detect_mask
