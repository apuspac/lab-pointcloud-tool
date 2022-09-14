#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <eigen3/Core>
#include <eigen3/SVD>


Eigen::Matrix3d calc_correlation_C(
    std::vector<Eigen::Vector3d> &x,
    std::vector<Eigen::Vector3d> &x_p, 
    double weight
    )
{
    // vector3d は double型の三次元ベクトル
    std::vector<Eigen::Vector3d> m, m_p;

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
    Eigen::Matrix3d correlation_C = Eigen::Matrix3d::Identity();

    //相関行列Cを求める
    // a,bの数が一緒であることが前提
    for (auto iter = std::begin(m), iter_p = std::begin(m_p), last = std::end(m);
         iter != last; ++iter, ++iter_p)
    {
        // std::cout << correlation_C << std::endl;
        // ここイテレータのまま.transpose()とか使うことできないのかな。シンプルになるんだけど。
        Eigen::Vector3d tmp = *iter;
        Eigen::Vector3d tmp_p = *iter_p;

        correlation_C += weight * tmp * (tmp_p.transpose());
    }


    return correlation_C;    
}

void SVD_correlation_C(Eigen::Matrix3d matrix_C)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> SVD(matrix_C, Eigen::ComputeFullU | Eigen::ComputeFullV);

    std::cout << "singular values" << std::endl;
    std::cout << SVD.singularValues() << std::endl;
    std::cout << "U matrix "  << std::endl;
    std::cout << SVD.matrixU() << std::endl;
    std::cout << "V matrix "  << std::endl;
    std::cout << SVD.matrixV() << std::endl;

    // 元の行列になるか。
    Eigen::Matrix3d M2;
    M2 = SVD.matrixU() * SVD.singularValues().asDiagonal() * SVD.matrixV().transpose();

    std::cout << "Matrix M2 = " << M2 << std::endl;

}

int main()
{
    // 仮データ準備
    std::vector<Eigen::Vector3d> x, x_p;
    
    Eigen::Vector3d a = {1.0, 2.0, 3.0};
    x.push_back(a);
    x.push_back(a);
    x.push_back(a);

    Eigen::Vector3d b = {2.0, 2.0, 3.0};
    x_p.push_back(b);
    x_p.push_back(b);
    x_p.push_back(b);

    // 重み
    double weight = 1.0;
    //相関行列C
    Eigen::Matrix3d correlation_C;
    correlation_C = calc_correlation_C(x, x_p, weight);

    std::cout << correlation_C << std::endl;

    SVD_correlation_C(correlation_C);    

    return 0;
}
