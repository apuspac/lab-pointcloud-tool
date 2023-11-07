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
        // openCV 逆っぽいため こちらも反転しておく
        height = img.rows;
        width = img.cols;
    }
}

void InstaImg::canny()
{
    cv::Mat output, tmp;
    // cv::Laplacian(img, tmp, CV_32F, 3);
    // cv::GaussianBlur(img, tmp, cv::Size(3, 3), 3, 3);

    cv::Canny(img, tmp, 50, 200, 3, true);
    img_edge = tmp;

    cv::convertScaleAbs(tmp, output, 1, 0);
    cv::resize(output, output, cv::Size(), 0.25, 0.25);

    cv::imshow("canny", output);
    cv::waitKey(0);
}

void LidarImg::edge_detect_sobel()
{
    cv::Mat output, sobel_x, sobel_y, sobel_tmp, tmp;

    // Cannyを使うとなぜだめなのかはメモ参照
    // cv::Canny(img_projected, tmp, 50, 200, 3, true);

    cv::GaussianBlur(img_projected, sobel_tmp, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);
    cv::Sobel(sobel_tmp, sobel_x, CV_8U, 1, 1, 3);
    cv::Sobel(sobel_tmp, sobel_y, CV_8U, 0, 1, 3);

    cv::add(sobel_x, sobel_y, tmp);
    // tmp = img_projected;
    img_edge = tmp;

    cv::convertScaleAbs(tmp, output, 1, 0);
    cv::resize(output, output, cv::Size(), 0.25, 0.25);

    cv::imshow("lidar_edge", output);
    cv::imwrite("lidar_edge.jpg", output);
    // cv::waitKey(0);
}

void InstaImg::convert_to_unitsphere(PointSet &projected_img)
{
    auto equirectangular_to_sphere = [=](double u, double v)
    {
        u /= width;
        v /= height;

        double phi = u * 2 * M_PI;
        double theta = v * M_PI;

        // REVIEW: ここ thetaにabsかけてもいいんですか？
        Eigen::Vector3d p = {abs(sin(theta)) * sin(phi), abs(sin(theta)) * cos(phi), cos(theta)};

        return p;
    };

    std::cout << "convert_to_unitsphere" << std::endl;

    // cv::Mat gray_img;
    // cv::cvtColor(img_edge, gray_img, cv::COLOR_BGR2GRAY);

    for (int v = 0; v < img_edge.rows; v++)
    {
        for (int u = 0; u < img_edge.cols; u++)
        {
            uchar *ptr = img_edge.data + img_edge.step * v;
            if (ptr[u] != 0)
            {
                // static_cast<int>(ptr[u])
                // std::cout << v << "." << u << " " << std::endl;
                projected_img.add_point(equirectangular_to_sphere(v, u));
            }
        }

        // std::cout << width << " " << height << std::endl;
    }

    // uchar *ptr = image.data + image.step * y;
    // uchar Blue = ptr[n * x];
    // uchar Green = ptr[n * x + 1];
    // uchar Red = ptr[n * x + 2];

    // img.forEach<u_char>([&projected_img, equirectangular_to_sphere](u_char &p, const int *position) -> void
    //                     {
    //     // double x = position[0];
    //     // double y = position[1];
    //     // projected_img.add_point(equirectangular_to_sphere(p[0], p[1]));
    //     // std::cout << p[0]  << " " << p[1] << " " << p[2] << std::endl;
    //     std::cout << p << " " << std::endl; });
}

void InstaImg::img_alpha_blending(const cv::Mat &blend_a, const cv::Mat &blend_b, double weight)
{

    cv::Mat out_a = cv::Mat::zeros(blend_a.rows, blend_a.cols, CV_8UC3);
    cv::Mat out_b = cv::Mat::zeros(blend_b.rows, blend_b.cols, CV_8UC3);

    (blend_a.channels() == 1) ? cv::cvtColor(blend_a, out_a, cv::COLOR_GRAY2BGR) : blend_a.copyTo(out_a);
    (blend_b.channels() == 1) ? cv::cvtColor(blend_b, out_b, cv::COLOR_GRAY2BGR) : blend_b.copyTo(out_b);

    for (int v = 0; v < img_edge.rows; v++)
    {
        for (int u = 0; u < img_edge.cols; u++)
        {
            uchar *ptr = img_edge.data + img_edge.step * v;
            if (ptr[u] != 0)
            {
                // blend_a greeen
                out_a.at<cv::Vec3b>(v, u)[0] = 20;
                out_a.at<cv::Vec3b>(v, u)[1] = 255;
                out_a.at<cv::Vec3b>(v, u)[2] = 181;
            }
            // blend_b white
            // out_b.at<cv::Vec3b>(v, u)[0] = 255;
            // out_b.at<cv::Vec3b>(v, u)[1] = 255;
            // out_b.at<cv::Vec3b>(v, u)[2] = 255;
        }
    }
    // alpha blending
    cv::Mat output = cv::Mat::zeros(blend_a.rows, blend_a.cols, CV_8UC3);
    // cv::addWeighted(out_a, weight, out_b, 1.0 - weight, 0, output);
    cv::addWeighted(out_a, 1.0, out_b, 1.0, 0, output);
    cv::resize(output, output, cv::Size(), 0.25, 0.25);

    // show
    cv::imshow("alpha blending", output);
    cv::imwrite("output_image.jpg", output);
    cv::waitKey(0);
}

double InstaImg::compute_MSE(const cv::Mat &reference, const cv::Mat &comparison)
{
    // std::cout << reference.size() << " " << comparison.size() << std::endl;
    // std::cout << reference.channels() << " " << comparison.channels() << std::endl;

    cv::Scalar_<double> dest = cv::quality::QualityMSE::compute(reference, comparison, cv::noArray());
    cv::Scalar_<double> dest_ssim = cv::quality::QualitySSIM::compute(reference, comparison, cv::noArray());

    // std::cout << dest[0] << " " << dest_ssim[0] << std::endl;
    return dest[0];
}