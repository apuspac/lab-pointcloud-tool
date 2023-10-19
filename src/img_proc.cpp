/**
 * @file img_proc.cpp
 * @brief
 *
 *
 */

#include "img_proc.hpp"

void InstaImg::load_img(std::string img_path)
{
    img = cv::imread(img_path);
    if (img.empty())
    {
        std::cout << "Could not open or find the image" << std::endl;
        std::exit(-1);
    }
    else
    {
        std::cout << "Success load image: " << img_path << std::endl;
        name = img_path;
        height = img.cols;
        width = img.rows;
    }
}

void InstaImg::canny()
{
    cv::Mat output, tmp, tmp2;
    // cv::Laplacian(img, tmp, CV_32F, 3);
    cv::GaussianBlur(img, tmp, cv::Size(3, 3), 3, 3);

    cv::Canny(tmp, tmp2, 50, 200, 3, true);
    cv::convertScaleAbs(tmp2, output, 1, 0);
    cv::resize(output, output, cv::Size(), 0.25, 0.25);

    img_edge = output;

    cv::imshow("canny", output);
    cv::waitKey(0);
}

void InstaImg::convert_to_unitsphere(PointSet &projected_img)
{
    auto equirectangular_to_sphere = [=](double x, double y)
    {
        x /= width;
        y /= height;

        double phi = x * 2 * M_PI;
        double theta = y * M_PI;

        Eigen::Vector3d p = {abs(sin(theta)) * cos(phi), abs(sin(theta)) * sin(phi), cos(theta)};

        return p;
    };

    std::cout << "convert_to_unitsphere" << std::endl;

    img.forEach<u_char>([&projected_img, equirectangular_to_sphere](u_char &p, const int *position) -> void
                        {
        // double x = position[0];
        // double y = position[1];
        // projected_img.add_point(equirectangular_to_sphere(p[0], p[1]));
        // std::cout << p[0]  << " " << p[1] << " " << p[2] << std::endl;
        std::cout << p << " " << std::endl; });
}