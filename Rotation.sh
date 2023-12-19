#!/bin/sh

./src/build/Rotation \
--img_cp img.dat \
--ply_cp ply.dat \
--ply Frame_20230711_nowall.ply \
--img img/PIC_20230711.jpg \
--dir data/projection/ \
--json data/20230711-bbox/20240711-bbox.json \
--mode 9
