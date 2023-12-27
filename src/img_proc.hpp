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
 * Instaとか言ってるが、メインの画像クラス
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
    InstaImg(std::string _name) : name(_name) {}
    virtual ~InstaImg() { img.release(); }

    // 入出力
    void load_img(std::string);

    // get
    std::array<int, 2> get_img_size() { return {height, width}; };
    std::string get_name() { return name; };
    int get_height() { return height; };
    int get_width() { return width; };
    cv::Mat get_mat() { return img; };

    // set
    void set_height(int _height) { height = _height; };
    void set_width(int _width) { width = _width; };
    void set_mat(cv::Mat _mat)
    {
        img = _mat;
        height = img.rows;
        width = img.cols;
    };
    void set_zero_imgMat(int, int, int);
    void set_pixel_255(int, int);

    // 表示
    void show(std::string, double);
    void show(std::string, double, const cv::Mat);
    void check_pixel_value();

    // 画像処理
    cv::Mat shift(int, int);
    void convert_to_unitsphere(PointSet &);
    void img_alpha_blending(const cv::Mat &, const cv::Mat &, double);
    double compute_MSE(const cv::Mat &, const cv::Mat &);
    void dilation(int, int);
    void erosion(int, int);
    void closing(int, int, int);
    void opening(int, int, int);
    void gaussian_blur(int);
    void diff_pixel(const cv::Mat &);
    void diff_img(const cv::Mat &);
};

/**
 * @brief edge画像クラス
 * 基本的に前提はCV_8UC1の二値画像
 *
 */
class EdgeImg : public InstaImg
{
private:
public:
    EdgeImg() : InstaImg() {}
    EdgeImg(std::string _name) : InstaImg(_name) {}
    ~EdgeImg() {}
    void detect_edge_with_canny(const cv::Mat &);
    void detect_edge_with_sobel(const cv::Mat &);
};

/**
 * @brief LiDAR画像クラス
 *
 *
 */
class LidarImg : public InstaImg
{
private:
    std::vector<std::vector<std::vector<Eigen::Vector3d>>> store_info;

public:
    LidarImg() : InstaImg() { store_info.resize(width, std::vector<std::vector<Eigen::Vector3d>>(height, std::vector<Eigen::Vector3d>())); }

    LidarImg(std::string _name) : InstaImg(_name) {}
    ~LidarImg() {}

    void set_store_info(int x, int y, Eigen::Vector3d point_data) { store_info[x][y].push_back(point_data); }
    const Eigen::Vector3d &get_store_info(int x, int y, int index) { return store_info[x][y][index]; }
    const std::vector<std::vector<std::vector<Eigen::Vector3d>>> &get_store_info() const { return store_info; }

    void ply_to_360paranoma_img(PointSet &);
    void ply_to_360paranoma_img(PointSet &, int);
};

#endif