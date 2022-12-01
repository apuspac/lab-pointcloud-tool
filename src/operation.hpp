/**
 * @file operation.hpp
 * @brief
 */

#ifndef OPERATION_HPP_INCLUDE_GUARD
#define OPERATION_HPP_INCLUDE_GUARD

#include "pointset.hpp"
#include "object_io.hpp"
#include "calc_pointset.hpp"

//相互依存
class ObjectIO;

/**
 * @brief オプション処理をして、ファイル名等を格納。その後の処理を決定する
 *
 */
class PointOperation
{
private:
    int mode;

    std::vector<std::string> corresp_img_file_name;
    std::vector<std::string> corresp_ply_file_name;
    std::vector<std::string> ply_file_name;
    std::vector<std::string> img_file_path;
    std::string default_dir_path;

public:
    std::string get_corresp_img_file_name(int number) { return corresp_img_file_name.at(number); }
    std::string get_corresp_ply_file_name(int number) { return corresp_ply_file_name.at(number); }
    std::string get_ply_file_path(int number) { return ply_file_name.at(number); }
    std::string get_img_file_path(int number) { return img_file_path.at(number); }
    std::string get_default_dir_path() { return default_dir_path; }

    void add_corresp_img_file_name(std::string name) { corresp_img_file_name.push_back(name); }
    void add_corresp_ply_file_name(std::string name) { corresp_ply_file_name.push_back(name); }
    void add_plyfile_name(std::string name) { ply_file_name.push_back(name); }
    void add_img_file_path(std::string name) { img_file_path.push_back(name); }
    void set_default_dir_path(std::string name) { default_dir_path = name; }

    void print();

    //ここに実行listを作ってmodeで切り替えしたい。
    void transform_coordinate();
    void Rotation_point();
    void Rotation_point_simlation();
};

#endif
