/**
 * @file operation.hpp
 * @brief
 */

#ifndef OPERATION_HPP_INCLUDE_GUARD
#define OPERATION_HPP_INCLUDE_GUARD

#include "pointset.hpp"
#include <unistd.h>

/**
 * @brief オプション処理をして、その後の処理を決定する
 *
 */
class PointOperation
{
private:
    int mode;
    std::vector<std::string> img_file_path;
    std::vector<std::string> point_file_path;
    std::vector<std::string> ply_file_path;
    std::string default_dir_path;

public:
    int option_process(int, char **);
    std::string get_point_file_path(int number) { return point_file_path.at(number); }
};

/**
 * @brief imgfile pointfile plyfileの読み込み
 *
 */
class PointSetIO
{
public:
    void print();
};

#endif
