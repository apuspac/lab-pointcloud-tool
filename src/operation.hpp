/**
 * @file operation.hpp
 * @brief
 */

#ifndef OPERATION_HPP_INCLUDE_GUARD
#define OPERATION_HPP_INCLUDE_GUARD

#include "pointset.hpp"
#include <fstream>
#include <unistd.h>
#include <getopt.h>

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
    std::vector<std::string> ply_file_path;
    std::vector<std::string> img_file_path;
    std::string default_dir_path;

public:
    int option_process(int, char **);
    std::string get_corresp_img_file_path(int number) { return corresp_img_file_name.at(number); }
    std::string get_corresp_ply_file_path(int number) { return corresp_ply_file_name.at(number); }
    std::string get_ply_file_path(int number) { return ply_file_path.at(number); }
    std::string get_img_file_path(int number) { return img_file_path.at(number); }
    std::string get_default_dir_path() { return default_dir_path; }

    void print();
};

/**
 * @brief imgfile pointfile plyfileの読み込み
 *
 */
class PointSetIO
{
public:
    void print();
    void load_ply_file(std::string, std::string, int, PointSet *);
};

#endif
