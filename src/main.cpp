#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <eigen3/Core>

void pointcloud_rotation()
{
    // vector3d は double型の三次元ベクトル
    std::vector<Eigen::Vector3d> x, x_p, m, m_p;

    Eigen::Vector3d a = {1.0, 2.0, 3.0};
    x.push_back(a);
    x.push_back(a);
    x.push_back(a);

    Eigen::Vector3d b = {2.0, 2.0, 3.0};
    x_p.push_back(b);
    x_p.push_back(b);
    x_p.push_back(b);

    // // 単位ベクトルに変換する
    for (const Eigen::Vector3d &point_to_vec : x)
    {
        m.push_back(point_to_vec.normalized());
        // std::cout << point_to_vec.normalized() << std::endl;
    }
    for (const Eigen::Vector3d &point_to_vec : x_p)
    {
        m_p.push_back(point_to_vec.normalized());
        // std::cout << point_to_vec.normalized() << std::endl;
    }

    for (const auto e : m)
    {
        std::cout << e << std::endl;
        std::cout << std::endl;
    }
}

int main()
{
    pointcloud_rotation();

    return 0;
}
