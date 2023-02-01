#ifndef CAPTURE_BOXPOINT_HPP_INCLUDE_GUARD
#define CAPTURE_BOXPOINT_HPP_INCLUDE_GUARD

#include <fstream>
#include <filesystem>
#include <cmath>

#include "pointset.hpp"

// nlohmann/json が手っ取り早く簡単かも

/**
 * @brief バウンディングボックス扱う用。
 *
 */
class BBox
{
private:
    double xmin; // 左上のx
    double ymin; // 左上のy
    double xmax; // 右下のx
    double ymax; // 右下のy

public:
    BBox(double x_min, double y_min, double x_max, double y_max) : xmin(x_min), ymin(y_min), xmax(x_max), ymax(y_max) {}
    ~BBox() {}
};

/**
 * @brief 画像点を読み込んで該当する点群をキャプチャ、抽出して保存するクラス
 *
 */
class CaptureBoxPoint
{
private:
    std::vector<BBox> bbox_list;

public:
    void capture_bbox(PointSet &, PointSet &, PointSet &, PointSet &);
    void capture_segmentation_distance(PointSet &, PointSet &, PointSet &, PointSet &);
    void capture_segmentation_angle(PointSet &, PointSet &, PointSet &, PointSet &);

    // void Load_json();
    void set_bbox(double, double, double, double);
};

#endif
