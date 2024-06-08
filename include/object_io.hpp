/**
 * @file object_io.hpp
 * @brief ファイルのIO
 */

#ifndef OBJECT_IO_HPP_INCLUDE_GUARD
#define OBJECT_IO_HPP_INCLUDE_GUARD

#include "pointset.hpp"
#include "operation.hpp"
#include "capture_boxpoint.hpp"
#include "img_proc.hpp"
#include <fstream>
#include <unistd.h>
#include <getopt.h>
#include <opencv2/opencv.hpp>

#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <cstdio>
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
<<<<<<< Updated upstream:src/object_io.hpp
    void load_ply_point_file(std::string, std::string, int, PointSet &);
    void load_img_point_file(std::string, std::string, std::string, PointSet &);
    static void output_ply(PointSet &, std::string);
=======
    void load_ply_point_file(int, PointSet &,std::string);
    void load_img_point_file(std::string, PointSet &, std::string);
    static void create_dir(std::string);
    void output_ply(PointSet &, std::string);
    // void output_ply(PointSet &);
>>>>>>> Stashed changes:include/object_io.hpp
    Eigen::Vector3d extend_distance_from_point_and_origin(Eigen::Vector3d, double);
    int load_detection_json_file(std::string, DetectionData &, std::string);
    void output_csv_2double(std::string, std::vector<std::vector<double>>);
    void output_dat(std::string, std::vector<Eigen::Vector3d>);
    void output_dat(std::string, std::vector<std::pair<int, int>>);
    void output_dat(std::string, std::vector<double>);
    void output_dat(std::string, std::vector<int>);

};

#endif
