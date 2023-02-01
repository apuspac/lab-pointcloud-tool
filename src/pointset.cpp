/**
 * @file pointset.cpp
 * @brief 点群を扱うpointsetの実装
 */
#include "pointset.hpp"

/**
 * @brief print
 *
 */
void PointSet::print()
{
    std::cout << "Point: " << name << std::endl;

    for (const auto &tmp : point3)
    {
        std::cout << std::setprecision(15) << tmp.transpose() << std::endl;
    }
    std::cout << std::endl;
    for (const auto &tmp : edge2)
    {
        std::cout << tmp.at(0) << " " << tmp.at(1) << std::endl;
    }
}

/**
 * @brief pointに回転行列を適用させる
 *
 * @param rotate_matrix matrix3d
 */
void PointSet::rotate(Eigen::Matrix3d rotate_matrix)
{
    std::cout << name << " point_rotated:" << rotate_matrix << std::endl;
    for (Eigen::Vector3d &tmp : point3)
    {
        tmp = rotate_matrix * tmp;
    }
}

/**
 * @brief pointに並進を適用させる
 *
 * @param transform_vec Vector3d
 */
void PointSet::transform(Eigen::Vector3d transform_vec)
{
    std::cout << name << " point_transformed:" << transform_vec.transpose() << std::endl;
    for (Eigen::Vector3d &tmp : point3)
    {
        tmp = transform_vec + tmp;
    }
}
