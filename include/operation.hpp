/**
 * @file operation.hpp
 * @brief 実際のプログラムの処理
 */

#ifndef OPERATION_HPP_INCLUDE_GUARD
#define OPERATION_HPP_INCLUDE_GUARD

#include "pointset.hpp"
#include "object_io.hpp"
#include "calc_pointset.hpp"
#include "capture_boxpoint.hpp"
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
    std::vector<std::string> corresp_img_file_name;
    // plyファイル対応点 ファイル名
    std::vector<std::string> corresp_ply_file_name;
    std::vector<std::string> ply_file_name;
    std::vector<std::string> img_file_path;
    // デフォルトdir(1つのフォルダに上のplyファイルをまとめる)
    std::string default_dir_path;
    std::string date;
    // jsonファイル名
    std::string json_file_path;

public:
    PointOperation(std::string _mode = "0") : mode(_mode) {}
    ~PointOperation() {}

    // ファイル名取得
    std::string get_corresp_img_file_name(int number) { return corresp_img_file_name.at(number); }
    std::string get_corresp_ply_file_name(int number) { return corresp_ply_file_name.at(number); }
    std::string get_ply_file_path(int number) { return ply_file_name.at(number); }
    std::string get_img_file_path(int number) { return img_file_path.at(number); }
    std::string get_default_dir_path() { return default_dir_path; }
    std::string get_json_path() { return json_file_path; }

    // モード取得
    int get_mode() { return std::stoi(mode); }
    void mode_select();
    std::string get_localtime();

    // set
    void set_corresp_img_file_name(std::string name) { corresp_img_file_name.push_back(name); }
    void set_corresp_ply_file_name(std::string name) { corresp_ply_file_name.push_back(name); }
    void set_plyfile_name(std::string name) { ply_file_name.push_back(name); }
    void set_img_file_path(std::string name) { img_file_path.push_back(name); }
    void set_default_dir_path(std::string name) { default_dir_path = name; }
    void set_json_path(std::string name) { json_file_path = name; }
    void set_mode(std::string mode_) { mode = mode_; }
    void set_date(std::string date_) { date = date_; }
    void set_date();
    void create_output_dir();

    // ファイル名を出力
    void print();

    // ここに実行listを作ってmodeで切り替えしたい。
    void transform_rotate();
    void transform_rotate_simulation();
    void rotate();
    void rotate_simulation();
    void capture_boxpoint();
    void capture_segmentation_point();
    void capture_pointset();
    void capture_point_inner_bbox();
    void old_detection_correspoint();
    void shift_test_w_stripe_pattern();
    void test_location();

    // mode切り替えしない関数
    void remove_pointset_floor(PointSet &, PointSet &, Eigen::Vector3d);

    // FIXME: 後でちゃんと関数化しよう
    void projection_to_sphere();
    // switch文回避のための map
    // typedef void (PointOperation::*mode_func)();
    std::unordered_map<int, std::function<void(void)>> switch_func;
};

#endif
