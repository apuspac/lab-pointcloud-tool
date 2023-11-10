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

/**
 * @brief メインの画像クラス
 * //HACK: Instaとか言ってるが、メインの画像クラス
 * こんがらがるだろ やめとけ やめとけ
 */
class InstaImg
{
protected:
    cv::Mat img;

    // cols, rows
    int height;
    int width;

    std::string name;

public:
    InstaImg() : img(cv::Mat()), height(0), width(0), name("none") {}
    InstaImg(int _width, int _height) : height(_height), width(_width), name("none") {}
    virtual ~InstaImg() { img.release(); }

    void load_img(std::string);
    std::array<int, 2> get_img_size() { return {height, width}; };
    std::string get_name() { return name; };
    int get_height() { return height; };
    int get_width() { return width; };
    cv::Mat get_mat() { return img; };

    void set_height(int _height) { height = _height; };
    void set_width(int _width) { width = _width; };

    void show(std::string, double);
    // 画像処理
    void shift(int, int, cv::Mat &);
    void convert_to_unitsphere(PointSet &);
    void img_alpha_blending(const cv::Mat &, const cv::Mat &, double);
    double compute_MSE(const cv::Mat &, const cv::Mat &);
    void set_pixel_255(int, int);
};

/**
 * @brief edge画像クラス
 *
 */
class EdgeImg : public InstaImg
{
private:
public:
    EdgeImg() : InstaImg() {}
    ~EdgeImg() {}
    void canny(cv::Mat);
}

/**
 * @brief LiDAR画像クラス
 *
 *
 */
class LidarImg : public InstaImg
{
    // private:
    //     cv::Mat img_projected;

public:
    LidarImg() : InstaImg() {}
    ~LidarImg() {}

    void set_zero_img_projected(int _height, int _width)
    {
        img = cv::Mat::zeros(_height, _width, CV_8UC1);
        set_height(_height);
        set_width(_width);
    }

    void ply_to_img(PointSet &);
    void detect_pointedge_with_sobel();
};
#endif