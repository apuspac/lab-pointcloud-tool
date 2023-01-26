#ifndef CAPTURE_BOXPOINT_HPP_INCLUDE_GUARD
#define CAPTURE_BOXPOINT_HPP_INCLUDE_GUARD

#include <fstream>
#include <filesystem>
#include <cmath>

#include "pointset.hpp"

// nlohmann/json が手っ取り早く簡単かも

class BBox
{
private:
    double xmin;
    double ymin;
    double xmax;
    double ymax;

public:
    BBox(double x_min, double y_min, double x_max, double y_max) : xmin(x_min), ymin(y_min), xmax(x_max), ymax(y_max) {}
    ~BBox() {}
};

class CaptureBoxPoint
{
private:
    // おそらく複数必要なので 格納させたほうがいい気がする。
    std::vector<BBox> bbox_list;

public:
    void test_check_process(PointSet &, PointSet &, PointSet &);
    void capture_segmentation(PointSet &, PointSet &, PointSet &, PointSet &);

    // void Load_json();
    void set_bbox(double, double, double, double);
};

#endif
