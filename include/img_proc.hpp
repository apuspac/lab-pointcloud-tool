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
    double compute_MSE(const std::vector<std::vector<int>> &, const std::vector<std::vector<int>> &);
    void dilation(int, int);
    void erosion(int, int);
    void closing(int, int, int);
    void opening(int, int, int);
    void gaussian_blur(int);
    void diff_pixel(const cv::Mat &);
    void diff_img(const cv::Mat &);
    std::vector<cv::Vec4i> HoughLine_vertical(int, double, double);

    void make_thetaphiIMG_from_pointcloud(PointSet &, std::pair<int, int>, bool gaussian_flag=false);
    void make_thetaphiIMG_from_panorama(std::pair<int, int>, bool gaussian_flag=false);

    void make_test_img_forEdge(int, int, int, int);
};

/**
 * @brief edgeを扱うための画像クラス
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

struct projection_info
{
    // v * width + u  形式
    int pixel;
    Eigen::Vector3d point;
    double distance;
};

/**
 * @brief LiDAR画像クラス
 *
 *
 */
class LidarImg : public InstaImg
{
private:

    // 投影された点がどの画素に投影されたか 画素座標(u + v * width)で格納
    // 画素ごとに格納する点は一つ
    std::vector<int> store_info;
    std::vector<projection_info> store_pixel;

public:
    LidarImg() : InstaImg() {}

    LidarImg(std::string _name) : InstaImg(_name) {}
    ~LidarImg() {}


    void resize_store_info(size_t point_num)
    {
        store_info.resize(point_num);
    }

    void set_store_info(int pixel, size_t index) { store_info.at(index) = pixel; }
    void set_store_pixel(int, Eigen::Vector3d);
    int get_store_info(int point_index) { return store_info.at(point_index); }

    void ply_to_360paranoma_img(PointSet &);
    void ply_to_360paranoma_img(PointSet &, int);
    void ply_to_360paranoma_img_depth(PointSet &, int);

    void get_corresponding_point(std::vector<Eigen::Vector3d> &, std::vector<std::pair<int, int>> &, EdgeImg &, EdgeImg &, PointSet &, int);
    void get_corresponding_point_Hough_old(std::vector<Eigen::Vector3d> &, std::vector<std::pair<int, int>> &, EdgeImg &, EdgeImg &, PointSet &, int, std::string);
    void get_corresponding_point_Hough(std::vector<Eigen::Vector3d> &, std::vector<std::pair<int, int>> &, EdgeImg &, EdgeImg &, PointSet &, int, std::string);

    bool is_equal_zero(double);
};

/**
 * methodのみを呼び出す形で使用。
 *
 */
class ImgCalc
{
public:
    static double compute_MSE(const std::vector<int> &, const std::vector<int> &);
    static double compute_MSE(const cv::Mat &, const cv::Mat &);
    static std::vector<int> shift(std::vector<int>, int, int, int, int);
    static cv::Mat point_to_img(PointSet &, std::pair<int, int>);
};

#endif
