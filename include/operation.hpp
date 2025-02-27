/**
 * @file operation.hpp
 * @brief 実際のプログラムの処理
 */

#ifndef OPERATION_HPP_INCLUDE_GUARD
#define OPERATION_HPP_INCLUDE_GUARD

#include "pointset.hpp"
#include "object_io.hpp"
#include "calc_pointset.hpp"
#include "capture_point.hpp"
#include "img_proc.hpp"

#include <iomanip>

#ifdef OPEN3D_ENABLED
#include "viewer.hpp"
#include "open3d/Open3D.h"
#endif

#ifdef MATPLOTLIB_ENABLED
#include "matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif


#include <sstream>

// 相互依存
// HACK: これ書いておく必要ある？
class ObjectIO;

/**
 * @brief ファイル名等を格納し、その後の処理を決定・実行させる。
 *
 */
class PointOperation
{
private:
    // モード
    std::string mode;

    // 画像対応点 ファイル名
    std::vector<std::string> corresp_img_file_path;
    // plyファイル対応点 ファイル名
    std::vector<std::string> corresp_ply_file_path;
    std::vector<std::string> ply_file_path;
    std::vector<std::string> img_file_path;
    // outputdir
    std::string output_dir_path;
    std::string date;
    // jsonファイル名
    std::string json_file_path;

public:
    PointOperation(
            std::string _mode = "0", 
            std::string _output_dir_path = "", 
            std::string _json_file_path = ""
    ) : mode(_mode), output_dir_path(_output_dir_path), json_file_path(_json_file_path){}

    ~PointOperation() {}




    // ファイル名取得
    std::string get_corresp_img_file_name(int number) { return corresp_img_file_path.at(number); }
    std::string get_corresp_ply_file_name(int number) { return corresp_ply_file_path.at(number); }
    std::string get_ply_file_path(int number) { return ply_file_path.at(number); }
    std::string get_img_file_path(int number) { return img_file_path.at(number); }
    std::string get_output_dir_path() { return output_dir_path; }
    std::string get_json_path() { return json_file_path; }

    // モード取得
    int get_mode() { return std::stoi(mode); }
    void mode_select();
    std::string get_localtime();

    // set
    void set_corresp_img_file_name(std::string name) { corresp_img_file_path.push_back(name); }
    void set_corresp_ply_file_name(std::string name) { corresp_ply_file_path.push_back(name); }
    void set_plyfile_name(std::string name) { ply_file_path.push_back(name); }
    void set_img_file_path(std::string name) { img_file_path.push_back(name); }
    void set_output_dir_path(std::string name) { output_dir_path = name; }
    void set_json_path(std::string name) { json_file_path = name; }
    void set_mode(std::string mode_) { mode = mode_; }
    void set_date(std::string date_) { date = date_; }
    void set_date();
    void create_output_dir();

    // ファイル名を出力
    void print();

    void capture_bbox_point(PointSet &, DetectionData &, Viewer3D &);
    void capture_mask_point(PointSet &, DetectionData &, Viewer3D &);
    void remove_pointset_floor(PointSet &, PointSet &, Eigen::Vector3d);

    // switch文回避のための map
    std::unordered_map<int, std::function<void(void)>> switch_func;

    // mode指定する。
    void transform_rotate();
    void transform_rotate_simulation();
    void rotate();
    void rotate_simulation();
    void capture_pointset_one();
    void capture_point_bbox_multi();
    void old_detection_correspoint();
    void shift_test_w_stripe_pattern();
    void make_img_and_calc_mse();
    void make_img_and_calc_mse_height();
    void capture_segmentation_point();
    void test_location();

#ifdef OPEN3D_ENABLED
    void projection_to_sphere();
#endif
};

#endif
