#ifndef CAPTURE_BOXPOINT_HPP_INCLUDE_GUARD
#define CAPTURE_BOXPOINT_HPP_INCLUDE_GUARD

#include <fstream>
#include <filesystem>
#include <cmath>

#include "pointset.hpp"

/**
 * @brief バウンディングボックスを扱う用。
 *
 */
class BBox
{
private:
    double xmin; // 左上のx
    double ymin; // 左上のy
    double xmax; // 右下のx
    double ymax; // 右下のy

    // equirectangular から変更したもの
    // leftup, rightup, leftdown, rightdown
    std::vector<Eigen::Vector3d> sphere_xyz;

    int class_num;
    double confidence;
    std::string class_name;

public:
    BBox(double x_min, double y_min, double x_max, double y_max) : xmin(x_min), ymin(y_min), xmax(x_max), ymax(y_max) {}
    ~BBox() {}

    void set_BBox(double x_min, double y_min, double x_max, double y_max) { xmin = x_min, ymin = y_min, xmax = x_max, ymax = y_max; }
    void set_class_name(std::string name) { class_name = name; }
    void set_class_num(int num) { class_num = num; }
    void set_confidence(double num) { confidence = num; }
    void equirectangular_to_sphere(double, double);

    // こんな感じで分解できる
    // auto [a, b, c, d] = get_bbox();
    std::tuple<double, double, double, double> get_bbox() { return {xmin, ymin, xmax, ymax}; }
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> get_minmax_xyz() { return {sphere_xyz.at(0), sphere_xyz.at(1), sphere_xyz.at(2), sphere_xyz.at(3)}; }
    std::vector<Eigen::Vector3d> get_xyz() { return sphere_xyz; }
    void print();
};

/**
 * @brief 一枚の画像から検出できたBBOXすべてを扱う。
 *
 */
class BBoxData
{
private:
    std::vector<BBox> bbox;
    std::string img_name;

public:
    void add_bbox(BBox add_box) { bbox.push_back(add_box); }

    // だいたいbbox全体をloopで回すと思うので、 get_bboxで取得するようにしよう
    std::vector<BBox> get_bbox_all() { return bbox; }
    std::string get_img_name() { return img_name; }
    void set_bbox(BBox bbox_one_instance) { bbox.push_back(bbox_one_instance); }
    void set_img_name(std::string name) { img_name = name; }
};

class Mask
{
private:
    // bboxと違ってここでも もう一階層あるのに注意。
    std::vector<std::array<int, 2>> mask_uv;

    std::vector<Eigen::Vector3d> mask_xyz;
    int class_num;
    std::string class_name;

public:
    void add_mask(int u, int v)
    {
        std::array<int, 2> mask2 = {u, v};
        mask_uv.push_back(mask2);
    }

    std::vector<std::array<int, 2>> get_mask() { return mask_uv; }
    void set_class_name(std::string name) { class_name = name; }
    std::vector<Eigen::Vector3d> get_mask_xyz() { return mask_xyz; }
    void print();

    void equirectangular_to_sphere(double, double);
};

/**
 * @brief 一枚の画像によるマスクデータを扱う。
 *
 *
 */
class MaskData
{
private:
    std::vector<Mask> mask_data;
    std::string img_name;

public:
    std::vector<Mask> get_mask_data_all() { return mask_data; }
    std::string get_img_name() { return img_name; }
    void set_mask_data(Mask mask_one_instance) { mask_data.push_back(mask_one_instance); }
    void set_img_name(std::string name) { img_name = name; }
};

/**
 * @brief Detection結果のJSONファイルの内容を格納するクラス。
 *
 * ちょっと冗長かもしれないが、複数枚の画像を使ったときに 役に立つ気がする。
 *
 */
class DetectionData
{
private:
    //  今は一枚の画像に対してのみ扱うが、 ゆくゆくは複数枚の写真の結果を統合するなどが欲しい。
    std::vector<BBoxData> bbox_data;
    std::vector<MaskData> mask_data;

public:
    std::vector<BBoxData> get_bbox_data() { return bbox_data; }
    std::vector<MaskData> get_mask_data() { return mask_data; }
    void set_bbox_data(BBoxData one_img_bbox) { bbox_data.push_back(one_img_bbox); }
    void set_mask_data(MaskData one_img_mask) { mask_data.push_back(one_img_mask); }
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
    void capture_bbox(PointSet &, PointSet &, BBox &, PointSet &);
    void capture_segmentation_distance(PointSet &, PointSet &, PointSet &, PointSet &);
    void capture_segmentation_angle(PointSet &, PointSet &, PointSet &, PointSet &);

    // void Load_json();
    void set_bbox(double, double, double, double);
};

#endif
