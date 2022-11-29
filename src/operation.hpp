/**
 * @file operation.hpp
 * @brief
 */

#ifndef OPERATION_HPP_INCLUDE_GUARD
#define OPERATION_HPP_INCLUDE_GUARD

#include "pointset.hpp"
#include <unistd.h>

class PointOperation
{
private:
    int mode;
    std::string img_file_path;
    std::vector<std::string> point_file_path;
    std::string default_dir_path;

public:
    int option_process(int, char **);
    std::string get_point_file_path(int number) { return point_file_path.at(number); }
};

class PointSetIO
{
public:
    void print();
};

#endif
