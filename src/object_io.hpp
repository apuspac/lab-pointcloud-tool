/**
 * @file object_io.hpp
 * @brief ファイルのIO
 */

#ifndef OBJECT_IO_HPP_INCLUDE_GUARD
#define OBJECT_IO_HPP_INCLUDE_GUARD

#include "pointset.hpp"
#include "operation.hpp"
#include <fstream>
#include <unistd.h>
#include <getopt.h>
#include <opencv2/opencv.hpp>

// 相互依存?
class PointOperation;

/**
 * @brief imgfile pointfile plyfileの読み込み, 書き込み
 *
 */
class ObjectIO
{
private:
public:
    static void option_process(int, char **, PointOperation &);
    void load_ply_point_file(std::string, std::string, int, PointSet &);
    void load_img_point_file(std::string, std::string, std::string, PointSet &);
    static void output_ply(PointSet &, std::string);
    Eigen::Vector3d extend_distance_from_point_and_origin(Eigen::Vector3d, double);
};

#endif