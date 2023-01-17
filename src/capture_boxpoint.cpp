#include "capture_boxpoint.hpp"

void check_point_in_polygon()
{
    Eigen::Vector3d target_point = {2, 2, 2};

    std::vector<Eigen::Vector3d> triangle;
    Eigen::Vector3d tri_0 = {1, 0, 0};
    Eigen::Vector3d tri_1 = {0, 1, 0};
    Eigen::Vector3d tri_2 = {0, 0, 1};
    triangle.push_back(tri_0);
    triangle.push_back(tri_1);
    triangle.push_back(tri_2);

    Eigen::Vector3d edge1 = triangle.at(1) - triangle.at(0);
    Eigen::Vector3d edge2 = triangle.at(2) - triangle.at(0);

    Eigen::Vector3d tri_center = {
        tri_0(0) + tri_1(0) + tri_2(0) / 3.0,
        tri_0(1) + tri_1(1) + tri_2(1) / 3.0,
        tri_0(2) + tri_1(2) + tri_2(2) / 3.0,
    };

    Eigen::Vector3d ray = tri_center - target_point;

    // print
    std::cout << "target: " << std::endl
              << target_point << std::endl
              << "triangle: " << std::endl
              << triangle.at(0) << std::endl
              << triangle.at(1) << std::endl
              << triangle.at(2) << std::endl
              << "edge: " << std::endl
              << edge1 << std::endl
              << edge2 << std::endl
              << "tri_center: " << std::endl
              << tri_center << std::endl
              << "ray: " << std::endl
              << ray << std::endl;

    // check
}

void CaptureBoxPoint::test_print()
{
    std::cout << "capture_boxpoint" << std::endl;

    check_point_in_polygon();
}

// Eigen::Vector3d equirectangular_to_sphere(double u, double v, double img_width, double img_height)
// {
//     // 正規化
//     u /= img_width;
//     v /= img_height;

//     // 緯度経度計算
//     double phi = u * 2 * M_PI;
//     double theta = v * M_PI;

//     // 方向ベクトル
//     Eigen::Vector3d p = {abs(sin(theta)) * sin(phi), abs(sin(theta)) * cos(phi), cos(theta)};

//     return p;
// }

// /**
//  * @brief 入力されたpoint_dataを単位球投影した方向ベクトルに変換する。
//  *
//  * @param point_data
//  * @return PointSet
//  */
// PointSet CalcPointSet::conversion_ply_to_img_point(PointSet &point_data)
// {
//     PointSet img_point_data("conversion_unit_sphere");
//     for (auto tmp : point_data.get_point_all())
//     {

//         if (check_float_equal(tmp(0), 0) && check_float_equal(tmp(1), 0) && check_float_equal(tmp(2), 0))
//         {
//             throw "0,0,0が含まれてます～！！！！！！！";
//         }

//         double theta = std::acos(
//             tmp(2) /
//             std::sqrt(std::pow(tmp(0), 2.0) + std::pow(tmp(1), 2.0) + std::pow(tmp(2), 2.0)));

//         double phi = std::atan2(tmp(1), tmp(0));

//         double r = 1.0;

//         // 方向ベクトル
//         Eigen::Vector3d p = {r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta)};

//         img_point_data.add_point(p);
//     }

//     return img_point_data;
// }

void CaptureBoxPoint::set_bbox(double xmin, double ymin, double xmax, double ymax)
{
    // "xmin":62.5458488464 "ymin":135.1288757324, "xmax":206.3257598877, "ymax":380.7976074219,
    // Eigen p1,p2,p3,p4;
    std::cout << xmin << ymin << xmax << ymax << std::endl;
}
