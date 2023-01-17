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
    // なんとか格納させる
    std::vector<std::vector<double>> bbox_list;

public:
    void test_print();
    // void Load_json();
    void set_bbox(double, double, double, double);
};

#endif
