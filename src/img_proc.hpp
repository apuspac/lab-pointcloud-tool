/**
 * @file img_proc.cpp
 * @brief 画像を主に扱いたい
 *
 */

#ifndef IMG_PROC_HPP_INCLUDE_GUARD
#define IMG_PROC_HPP_INCLUDE_GUARD

#include "object_io.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/quality.hpp>
#include <opencv2/core/types.hpp>

class InstaImg
{
protected:
    cv::Mat img;
    cv::Mat img_edge;

    // cols, rows
    int height;
    int width;

    std::string name;

public:
    InstaImg() : img(cv::Mat()), height(0), width(0), name("none") {}
    ~InstaImg() {}
    void load_img(std::string);
    std::array<int, 2> get_img_size() { return {height, width}; };
    std::string get_name() { return name; };
    int get_height() { return height; };
    int get_width() { return width; };
    cv::Mat get_mat() { return img; };
    cv::Mat get_mat_edge() { return img_edge; };

    void set_height(int _height) { height = _height; };
    void set_width(int _width) { width = _width; };

    void show() { cv::imshow(name, img); }
    // 画像処理
    void canny();
    void convert_to_unitsphere(PointSet &);
    void img_alpha_blending(const cv::Mat &, const cv::Mat &, double);
    double compute_MSE(const cv::Mat &, const cv::Mat &);
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
    void edge_detect_sobel();

    // override
    cv::Mat get_mat() { return img_projected; };

    // REVIEW: バイトサイズがCV_8UC1でいいかどうかあとでチェック
    void set_zero_img_projected(int _height, int _width)
    {
        img_projected = cv::Mat::zeros(_height, _width, CV_8UC1);
        set_height(_height);
        set_width(_width);
    }
    void set_point_projected(int x, int y)
    {
        img_projected.at<u_char>(y, x) = 255;
        // img_projected.at<cv::Vec3b>(y, x)[0] = 255;
        // img_projected.at<cv::Vec3b>(y, x)[1] = 255;
        // img_projected.at<cv::Vec3b>(y, x)[2] = 255;
    }

    void ply_to_img(PointSet &);
    void shift(int, int, cv::Mat &);
};
#endif