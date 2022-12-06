#include <iostream>
#include <eigen3/Core>
#include <opencv2/opencv.hpp>

int main()
{
    // eigen test
    Eigen::Vector3d a;

    // ベクトルの要素指定
    a << 1.0, 0.7, -0.52;

    //そのまま出力
    std::cout << "vector a = " << std::endl;
    std::cout << a << std::endl;
    //転置して出力
    std::cout << "vector a = " << a.transpose() << std::endl;

    std::cout << "a(0) = " << a(0) << std::endl;
    std::cout << "a(1) = " << a(1) << std::endl;
    std::cout << "a(2) = " << a(2) << std::endl;

    // cvtest
    cv::Mat img = cv::imread("img/aaa.png", -1);
    if (img.empty())
    {
        return -1;
    }
    cv::namedWindow("Example", cv::WINDOW_AUTOSIZE);
    cv::imshow("Example", img);
    cv::waitKey(0);
    cv::destroyWindow("Example");
}
