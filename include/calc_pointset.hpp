#ifndef CALC_POINTSET_HPP_INCLUDE_GUARD
#define CALC_POINTSET_HPP_INCLUDE_GUARD

#include <random>
#include <fstream>
#include <filesystem>
#include <cmath>

#include "pointset.hpp"

class CalcPointSet
{
public:
    void test_print();

    Eigen::Matrix3d calc_theory_value_Rotation_Matrix(Eigen::Vector3d, double);

    // 基本行列E -> 並進t -> 回転Rまで出す群
    std::vector<Eigen::Matrix<double, 9, 1>> create_xi_vector(PointSet &, PointSet &);
    Eigen::Matrix<double, 9, 9> calc_matrix_M(std::vector<Eigen::Matrix<double, 9, 1>> &);
    Eigen::Matrix3d calc_essential_matrix(Eigen::Matrix<double, 9, 9> &);
    Eigen::Vector3d calc_translation_t(Eigen::Matrix3d &);
    Eigen::Vector3d check_sign_translation_t(PointSet &, PointSet &, Eigen::Vector3d &, Eigen::Matrix3d &);
    Eigen::Matrix3d calc_rotation_matrix_from_essential_matrix(Eigen::Matrix3d &, Eigen::Vector3d &);
    double calc_scale_of_translation_t(PointSet &, PointSet &, Eigen::Matrix3d &, Eigen::Vector3d &);

    // 回転前後の点群を扱う
    Eigen::Matrix3d calc_correlation_C(PointSet &, PointSet &, double);
    Eigen::Matrix3d calc_rotation_matrix_from_correlation_c(Eigen::Matrix3d &);
    void calc_rotation_axis_from_matrix_R(Eigen::Matrix3d &);

    PointSet conversion_ply_to_img_point(PointSet &);
    void pickup_corresp_point(PointSet &, PointSet &, PointSet &, PointSet &, std::string="");

    void radius_based_outlier_filter(PointSet &, double);
    static void make_striped_pattern(std::vector<int> &, PointSet &, PointSet &, int );
};

#endif
