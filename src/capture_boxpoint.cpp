#include "capture_boxpoint.hpp"

void CaptureBoxPoint::test_print()
{
    std::cout << "capture_boxpoint" << std::endl;
}

Eigen::Vector3d equirectangular_to_sphere(double u, double v, double img_width, double img_height)
{
    // 正規化
    u /= img_width;
    v /= img_height;

    // 緯度経度計算
    double phi = u * 2 * M_PI;
    double theta = v * M_PI;

    // 方向ベクトル
    Eigen::Vector3d p = {abs(sin(theta)) * sin(phi), abs(sin(theta)) * cos(phi), cos(theta)};

    return p;
}

/**
 * @brief 入力されたpoint_dataを単位球投影した方向ベクトルに変換する。
 *
 * @param point_data
 * @return PointSet
 */
PointSet CalcPointSet::conversion_ply_to_img_point(PointSet &point_data)
{
    PointSet img_point_data("conversion_unit_sphere");
    for (auto tmp : point_data.get_point_all())
    {

        if (check_float_equal(tmp(0), 0) && check_float_equal(tmp(1), 0) && check_float_equal(tmp(2), 0))
        {
            throw "0,0,0が含まれてます～！！！！！！！";
        }

        double theta = std::acos(
            tmp(2) /
            std::sqrt(std::pow(tmp(0), 2.0) + std::pow(tmp(1), 2.0) + std::pow(tmp(2), 2.0)));

        double phi = std::atan2(tmp(1), tmp(0));

        double r = 1.0;

        // 方向ベクトル
        Eigen::Vector3d p = {r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta)};

        img_point_data.add_point(p);
    }

    return img_point_data;
}

void CaptureBoxPoint::set_bbox(double xmin, double ymin, double xmax, double ymax)
{
    // "xmin":62.5458488464 "ymin":135.1288757324, "xmax":206.3257598877, "ymax":380.7976074219,
    // Eigen p1,p2,p3,p4;
}
