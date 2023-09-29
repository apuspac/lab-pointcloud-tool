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

void PointSet::add_point(PointSet add_pointset)
{
    add_pointset.get_point_all();

    for (auto point : add_pointset.get_point_all())
    {
        point3.push_back(point);
    }

    // if (point3.capacity() < point3.size() + add_pointset.get_point_all().size())
    // {
    //     point3.reserve(point3.size() + add_pointset.get_point_all().size());
    // }
    // point3.insert(point3.end(), add_pointset.get_point_all().begin(), add_pointset.get_point_all().end());
    // std::cout << "oi" << std::endl;
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

/**
 * @brief pointsetの重心を返す
 *
 * @return Eigen::Vector3d
 */
Eigen::Vector3d PointSet::get_center_of_gravity()
{
    std::cout << "get_center_of_gravity" << std::endl;

    Eigen::Vector3d sum = std::reduce(point3.begin(), point3.end(), Eigen::Vector3d(0, 0, 0), [](Eigen::Vector3d a, Eigen::Vector3d b)
                                      { return a + b; });
    return (sum / static_cast<double>(point3.size()));
}