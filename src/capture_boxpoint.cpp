#include "capture_boxpoint.hpp"

bool check_point_in_polygon(Eigen::Vector3d target_point, Eigen::Vector3d tri_0, Eigen::Vector3d tri_1, Eigen::Vector3d tri_2)
{
    // Eigen::Vector3d target_point = {2, 2, 2};

    std::vector<Eigen::Vector3d> triangle;
    // Eigen::Vector3d tri_0 = {1, 0, 0};
    // Eigen::Vector3d tri_1 = {0, 1, 0};
    // Eigen::Vector3d tri_2 = {0, 0, 1};
    triangle.push_back(tri_0);
    triangle.push_back(tri_1);
    triangle.push_back(tri_2);

    Eigen::Vector3d edge1 = triangle.at(1) - triangle.at(0);
    Eigen::Vector3d edge2 = triangle.at(2) - triangle.at(0);

    Eigen::Vector3d triangle_center = {
        tri_0(0) + tri_1(0) + tri_2(0) / 3.0,
        tri_0(1) + tri_1(1) + tri_2(1) / 3.0,
        tri_0(2) + tri_1(2) + tri_2(2) / 3.0,
    };

    Eigen::Vector3d ray = triangle_center - target_point;

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
              << triangle_center << std::endl
              << "ray: " << std::endl
              << ray << std::endl;

    // check
    // |a,b,c|
    auto scalar_triple_product = [](Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c)
    {
        return (a.cross(b)).dot(c);
    };

    double fraction_down = scalar_triple_product(edge1, edge2, -ray);

    std::cout << "fraction_down: " << fraction_down << std::endl;

    bool flag = false;

    if (fraction_down > 0)
    {
        double u = scalar_triple_product(target_point - triangle.at(0), edge2, -ray) / fraction_down;
        double v = scalar_triple_product(edge1, target_point - triangle.at(0), -ray) / fraction_down;
        double s = scalar_triple_product(edge1, edge2, target_point - triangle.at(0)) / fraction_down;

        if ((u >= 0) && (u <= 1))
        {
            if ((v >= 0) && (v <= 1) && (u + v <= 1))
            {

                std::cout << "point in polygon" << std::endl;
                flag = true;
            }
        }

        std::cout << "u: " << u << std::endl
                  << "v: " << v << std::endl
                  << "s: " << s << std::endl;
    }

    return flag;
}

// void CaptureBoxPoint::make_square_pyramid

void CaptureBoxPoint::set_bbox(double xmin, double ymin, double xmax, double ymax)
{
    // "xmin":62.5458488464 "ymin":135.1288757324, "xmax":206.3257598877, "ymax":380.7976074219,
    // Eigen p1,p2,p3,p4;
    BBox square_pyramid(xmin, ymin, xmax, ymax);
    std::cout << xmin << ymin << xmax << ymax << std::endl;

    bbox_list.push_back(square_pyramid);
}

void CaptureBoxPoint::test_check_process(PointSet &plypoint, PointSet &capture_point, PointSet &bbox_point)
{
    std::cout << "capture_boxpoint" << std::endl;

    // Eigen::Vector3d target_point_1 = {2, 2, 2};
    // Eigen::Vector3d tri_0_1 = {1, 0, 0};
    // Eigen::Vector3d tri_1_1 = {0, 1, 0};
    // Eigen::Vector3d tri_2_1 = {0, 0, 1};
    // std::cout << check_point_in_polygon(target_point_1, tri_0_1, tri_1_1, tri_2_1) << std::endl;

    // ここから別

    // BBOX
    std::vector<Eigen::Vector3d> box;

    // test
    // Eigen::Vector3d box_0 = {1.0, 0, 0};
    // Eigen::Vector3d box_1 = {0, 1.0, 0};
    // Eigen::Vector3d box_2 = {1.0, 0, 1.0};
    // Eigen::Vector3d box_3 = {0, 1.0, 1.0};

    // Eigen::Vector3d box_0 = {-5.104841, -0.264854, -0.432836};
    // Eigen::Vector3d box_1 = {-5.250792, -2.375239, -0.737240};
    // Eigen::Vector3d box_2 = {-5.079898, -0.111731, 1.998436};
    // Eigen::Vector3d box_3 = {-5.252830, -2.340908, 1.711119};

    // Eigen::Vector3d box_0 = {0.0180694246632902, -0.989178343801835, 0.145601167735005};
    // Eigen::Vector3d box_1 = {0.200924346436544, -0.969558093056052, 0.139951817417851};
    // Eigen::Vector3d box_2 = {0.020467338986129, -0.896302221800321, 0.442971122343923};
    // Eigen::Vector3d box_3 = {0.188935604690031, -0.87640168989441, 0.442971122343923};

    /**
0.0180694246632902 -0.989178343801835  0.145601167735005
 0.200924346436544 -0.969558093056052  0.139951817417851
 0.020467338986129 -0.896302221800321  0.442971122343923
0.188935604690031 -0.87640168989441 0.442971122343923

  0.90092454372665 -0.433681410566534  0.015981259022634
 0.987333027171734 -0.156840374815621 0.0239706129165579
 0.861291266136286 -0.417027770092863  0.290284677254462
 0.933123850478427 -0.164656951975056  0.319637244129339

 -0.989208551514083, 0.0701305152071164,  0.128639622388256
 -0.98087710188616, -0.139782275846257,  0.135429045461631
-0.888296004751992, 0.0609382953025009,   0.45521064586329
-0.881478072948008, -0.125617175595506 ,  0.45521064586329

-0.989327229032126, 0.0565305750745415,  0.134297907568797
 -0.989431308208918, -0.0599367711367848,   0.132035108216282
     */

    Eigen::Vector3d box_0 = {-0.989327229032126, 0.0565305750745415, 0.134297907568797};
    Eigen::Vector3d box_1 = {-0.989431308208918, -0.0599367711367848, 0.132035108216282};
    Eigen::Vector3d box_2 = {-0.914145863707222, 0.0553762054667961, 0.401585377890028};
    Eigen::Vector3d box_3 = {-0.913352210360464, -0.0532354562838329, 0.403675272990442};

    box.push_back(box_0);
    box.push_back(box_1);
    box.push_back(box_2);
    box.push_back(box_3);

    bbox_point.add_point(box_0);
    bbox_point.add_point(box_1);
    bbox_point.add_point(box_2);
    bbox_point.add_point(box_3);

    std::cout << "box:" << std::endl
              << box.at(0).transpose() << std::endl
              << box.at(1).transpose() << std::endl
              << box.at(2).transpose() << std::endl
              << box.at(3).transpose() << std::endl;

    // 面法線を求める
    auto calc_plane_normal = [](std::vector<Eigen::Vector3d> triangle)
    {
        Eigen::Vector3d v0v1 = triangle.at(1) - triangle.at(0);
        Eigen::Vector3d v0v2 = triangle.at(2) - triangle.at(0);

        Eigen::Vector3d plane_normal = v0v1.cross(v0v2);
        plane_normal.normalize();

        return plane_normal;
    };

    // 一応dも求める
    auto calc_d = [](Eigen::Vector3d normal, Eigen::Vector3d point)
    {
        return (-(normal(0) * point(0) + normal(1) * point(1) + normal(2) * point(2)));
    };

    // 点を代入したときの距離が 0より上かどうかを判定する
    auto is_point_upper_side_of_plane = [](Eigen::Vector3d point, Eigen::Vector3d normal, double d)
    {
        double tmp = normal(0) * point(0) + normal(1) * point(1) + normal(2) * point(2) + d;

        std::cout << "point_upper:" << tmp << std::endl;

        if (tmp > 0)
        {
            return true;
        }
        else if (tmp < 0)
        {
            return false;
        }
        return false;
    };

    // double d = -(plane_normal(0) * triangle_1.at(1)(0) + plane_normal(1) * triangle_1.at(1)(1) + plane_normal(2) * triangle_1.at(1)(2));

    Eigen::Vector3d origin = {0, 0, 0};

    // 1つ目
    std::vector<Eigen::Vector3d> triangle_1;
    triangle_1.push_back(origin);
    triangle_1.push_back(box_0);
    triangle_1.push_back(box_1);

    Eigen::Vector3d normal_1 = calc_plane_normal(triangle_1);
    double d_1 = calc_d(normal_1, triangle_1.at(1));

    // 2つ目
    std::vector<Eigen::Vector3d> triangle_2;
    triangle_2.push_back(origin);
    triangle_2.push_back(box_2);
    triangle_2.push_back(box_0);

    Eigen::Vector3d normal_2 = calc_plane_normal(triangle_2);
    double d_2 = calc_d(normal_2, triangle_2.at(1));

    // 3つ目
    std::vector<Eigen::Vector3d> triangle_3;
    triangle_3.push_back(origin);
    triangle_3.push_back(box_1);
    triangle_3.push_back(box_3);

    Eigen::Vector3d normal_3 = calc_plane_normal(triangle_3);
    double d_3 = calc_d(normal_3, triangle_3.at(1));

    // 4つ目
    std::vector<Eigen::Vector3d> triangle_4;
    triangle_4.push_back(origin);
    triangle_4.push_back(box_3);
    triangle_4.push_back(box_2);

    Eigen::Vector3d normal_4 = calc_plane_normal(triangle_4);
    double d_4 = calc_d(normal_4, triangle_4.at(1));

    for (auto target_point : plypoint.get_point_all())
    {

        if (is_point_upper_side_of_plane(target_point, normal_1, d_1) && is_point_upper_side_of_plane(target_point, normal_2, d_2) && is_point_upper_side_of_plane(target_point, normal_3, d_3) && is_point_upper_side_of_plane(target_point, normal_4, d_4))
        {
            capture_point.add_point(target_point);
        }
    }

    std::cout
        << "plane_normal: " << normal_1.transpose() << std::endl
        << "d: " << d_1 << std::endl;
}

void capture_segmentation(PointSet &, PointSet &, PointSet &)
{
    std::cout << "capture_segmentation_point" << std::endl;
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
