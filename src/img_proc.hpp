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
private:
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

    void show() { cv::imshow(name, img); }
    // 画像処理
    void canny();
    void convert_to_unitsphere(PointSet &);
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