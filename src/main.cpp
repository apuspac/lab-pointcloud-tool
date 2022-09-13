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

    // 単位ベクトルに変換する
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

    //相関行列C
    double correlation_C = 0.0;

    //重み
    double weight = 1.0;

    //相関行列Cを求める
    // a,bのポイント数が一緒であることが前提
    for (auto iter = std::begin(m), iter_p = std::begin(m_p), last = std::end(m);
         iter != last; ++iter, ++iter_p)
    {
        // std::cout << correlation_C << std::endl;
        Eigen::Vector3d tmp = *iter;
        Eigen::Vector3d tmp_p = *iter_p;

        correlation_C += weight * tmp.dot(tmp_p.transpose());
    }

    std::cout << correlation_C << std::endl;
}

int main()
{
    pointcloud_rotation();

    return 0;
}
