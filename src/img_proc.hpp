/**
 * @file img_proc.cpp
 * @brief 画像を主に扱いたい
 *
 */

#ifndef IMG_PROC_HPP_INCLUDE_GUARD
#define IMG_PROC_HPP_INCLUDE_GUARD

#include "object_io.hpp"
#include <opencv2/opencv.hpp>

class InstaImg
{
protected:
    cv::Mat img;
    cv::Mat img_edge;

    // cols, rows
    double height;
    double width;

    std::string name;

public:
    InstaImg() : img(cv::Mat()), height(0), width(0), name("none") {}
    ~InstaImg() {}
    void load_img(std::string);
    std::array<double, 2> get_img_size() { return {height, width}; };
    std::string get_name() { return name; };
    double get_height() { return height; };
    double get_width() { return width; };
    cv::Mat get_mat() { return img; };
    cv::Mat get_mat_edge() { return img_edge; };

    void set_height(double _height) { height = _height; };
    void set_width(double _width) { width = _width; };

    void show() { cv::imshow(name, img); }
    // 画像処理
    void canny();
    void convert_to_unitsphere(PointSet &);
    void img_alpha_blending(const cv::Mat &, const cv::Mat &, double);
};

class LidarImg : public InstaImg
{
private:
    cv::Mat img_projected;

public:
    LidarImg() : InstaImg(), img_projected(cv::Mat()) {}
    ~LidarImg() {}
    void show() { cv::imshow(name, img_projected); }
    cv::Mat get_mat_projected() { return img_projected; };

    // REVIEW: バイトサイズがCV_8UC1でいいかどうかあとでチェック
    void set_zero_img_projected(double _height, double _width) { img_projected = cv::Mat::zeros(static_cast<int>(_height), static_cast<int>(_width), CV_8UC3); }
    void set_point_projected(int x, int y)
    {
        std::cout << x << " " << y << "::" << std::endl;
        img_projected.at<cv::Vec3b>(x, y)[0] = 255;
        img_projected.at<cv::Vec3b>(x, y)[1] = 255;
        img_projected.at<cv::Vec3b>(x, y)[2] = 255;
    }
};
#endif