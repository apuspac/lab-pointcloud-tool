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

    double set_height(double _height) { height = _height; };
    double set_width(double _width) { width = _width; };

    void show() { cv::imshow(name, img); }
    // 画像処理
    void canny();
    void convert_to_unitsphere(PointSet &);
};

class LidarImg : public InstaImg
{
private:
    cv::Mat img_projected;

public:
    LidarImg() : InstaImg(), img_projected(cv::Mat()) {}
    ~LidarImg() {}
    void show() { cv::imshow(name, img_projected); }
    void set_zero_img_projected(double _height, double _width) { img_projected = cv::Mat::zeros(_height, _width, CV_8UC1); }
    void set_point_projected(int x, int y) { img_projected.at<uchar>(x, y) = 255; }
};

/**
 * @brief こっちが複数のやつを扱う Imgは単体
 *
 * 必要かどうかは微妙なので作るだけ。
 *
 */
// class ImgProc
// {
//     class
// };

#endif