#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <eigen3/Core>
#include <eigen3/SVD>
#include <eigen3/LU>



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

Eigen::Matrix3d calc_rotation_R(Eigen::Matrix3d correlation_C)
{
    // 特異値分解
    Eigen::JacobiSVD<Eigen::Matrix3d> SVD(correlation_C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d matrix_V, matrix_U,  matrix_R;

    matrix_U = SVD.matrixU();
    matrix_V = SVD.matrixV();

    // det(V UT) 計算結果は 1か-1
    double det_VU = (matrix_V * matrix_U.transpose()).determinant();

    //対角行列
    Eigen::DiagonalMatrix<double, 3> matrix_Diag = {1.0, 1.0, det_VU};

    // 回転行列Rの最大化の式
    matrix_R = matrix_V * matrix_Diag * matrix_U.transpose();

    //check
    std::cout << "check" << std::endl;
    std::cout << matrix_R.transpose() * matrix_R << std::endl;
    std::cout << matrix_R * matrix_R.transpose() << std::endl;

    std::cout << "det" << std::endl;
    std::cout << matrix_R.determinant() << std::endl;
    
    return matrix_R;
}

void SVD_test(Eigen::Matrix3d matrix_C)
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

    Eigen::Vector3d a1 = {2.0, -1.0, 3.0};
    Eigen::Vector3d a2 = {2.0, -1.0, 0.0};
    Eigen::Vector3d a3 = {0.0, -1.0, 3.0};
    Eigen::Vector3d a4 = {0.0, -1.0, 0.0};
    Eigen::Vector3d a5 = {2.0, 0.0, 3.0};
    Eigen::Vector3d a6 = {2.0, 0.0, 0.0};
    
    
    Eigen::Vector3d b1 = {2.0, 1.0, 3.0};
    Eigen::Vector3d b2 = {2.0, 1.0, 0.0};
    Eigen::Vector3d b3 = {0.0, 1.0, 3.0};
    Eigen::Vector3d b4 = {0.0, 1.0, 0.0};
    Eigen::Vector3d b5 = {2.0, 0.0, 3.0};
    Eigen::Vector3d b6 = {2.0, 0.0, 0.0};

    x.push_back(a1);
    x.push_back(a2);
    x.push_back(a3);
    x.push_back(a4);
    x.push_back(a5);
    x.push_back(a6);

    x_p.push_back(b1);
    x_p.push_back(b2);
    x_p.push_back(b3);
    x_p.push_back(b4);
    x_p.push_back(b5);
    x_p.push_back(b6);

    // 重み
    double weight = 1.0;
    //相関行列C
    Eigen::Matrix3d correlation_C;
    correlation_C = calc_correlation_C(x, x_p, weight);

    std::cout << "correlation_C : " << std::endl;
    std::cout << correlation_C << std::endl;

    // SVD_test(correlation_C);    
    Eigen::Matrix3d matrix_R;
    matrix_R = calc_rotation_R(correlation_C);


    std::cout << "matrix_R :" << std::endl;
    std::cout <<  matrix_R << std::endl; 
    
    std::cout << "x1_prime : " << std::endl << b2 << std::endl; 
    std::cout << "R * x1 : " << std::endl << matrix_R * a2 << std::endl; 

    return 0;
}
