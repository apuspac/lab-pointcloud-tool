#!/bin/sh

./src/build/Rotation \
--img_cp img.dat \
--ply_cp ply.dat \
--ply out-1.ply \
--img img/ply2img_closing_sobel.png \
--dir data/20231213/ \
--json data/20230711-bbox/20240711-bbox.json \
--mode 3
# --img img/20231213/PIC_20231213_point1-120-hi.jpg \
