#include "capture_boxpoint.hpp"

void BBox::print()
{
    // std::cout << class_num << " " << class_name << std::endl;
    std::cout << class_name << std::endl;
    std::cout << xmin << " " << ymin << " " << xmax << " " << ymax << std::endl;
    std::cout << sphere_xyz.at(0).transpose() << std::endl
              << sphere_xyz.at(1).transpose() << std::endl
              << sphere_xyz.at(2).transpose() << std::endl
              << sphere_xyz.at(3).transpose() << std::endl
              << std::endl;
}

void Mask::print()
{
    for (const auto uv : mask_uv)
    {
        std::cout << uv.at(0) << " " << uv.at(1) << std::endl;
    }
}
/**
 * @brief 正距円筒図法(360度画像)の画素値から単位球に投影したとき方向ベクトルへ変換する。左上が原点でのuv座標形式の画素値が入力される前提。緯度経度を計算し、xyzに変換する。
 *
 * @param u 画素値の横座標
 * @param v 画素値の縦座標
 * @param img_width 360度画像の横
 * @param img_height 360度画像の縦
 * @return Eigen::Vector3d 変換したベクトルを返す
 */
Eigen::Vector3d equirec_to_sphere(double u, double v, double width, double height)
{
    // 正規化
    u /= width;
    v /= height;

    // 緯度経度計算
    double phi = u * 2 * M_PI;
    double theta = v * M_PI;

    // 方向ベクトルに変換
    Eigen::Vector3d p = {abs(sin(theta)) * sin(phi), abs(sin(theta)) * cos(phi), cos(theta)};

    return p;
}

void BBox::equirectangular_to_sphere(double img_width, double img_height)
{
    sphere_xyz.push_back(equirec_to_sphere(xmin, ymin, img_width, img_height));
    sphere_xyz.push_back(equirec_to_sphere(xmax, ymin, img_width, img_height));
    sphere_xyz.push_back(equirec_to_sphere(xmin, ymax, img_width, img_height));
    sphere_xyz.push_back(equirec_to_sphere(xmax, ymax, img_width, img_height));
}

void Mask::equirectangular_to_sphere(double img_width, double img_height)
{
    for (auto uv : get_mask())
    {
        Eigen::Vector3d p = equirec_to_sphere(uv.at(0), uv.at(1), img_width, img_height);
        mask_xyz.push_back(p);
    }
}

/**
 * @brief 対象点がpointが3角形の中に入っているかどうかを判定する。 内外判定
 * https://shikousakugo.wordpress.com/2012/06/27/ray-intersection-2/
 * (使っていない)
 *
 * @param target_point
 * @param tri_0
 * @param tri_1
 * @param tri_2
 * @return true
 * @return false
 */
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

/**
 * @brief バウンディングボックスを追加。 TODO: Setbboxはbboxクラスに実装すべき
 *
 * @param xmin
 * @param ymin
 * @param xmax
 * @param ymax
 */
void CaptureBoxPoint::set_bbox(double xmin, double ymin, double xmax, double ymax)
{
    // "xmin":62.5458488464 "ymin":135.1288757324, "xmax":206.3257598877, "ymax":380.7976074219,
    // Eigen p1,p2,p3,p4;
    BBox square_pyramid(xmin, ymin, xmax, ymax);
    std::cout << xmin << ymin << xmax << ymax << std::endl;

    bbox_list.push_back(square_pyramid);
}

/**
 * @brief バウンディングボックス内の点群を抽出する、一つのBBoxに対して行う。

 *
 * @param plypoint 抽出対象の点群
 * @param capture_point 抽出した点群を格納する
 * @param detect_bbox バウンディングボックスの点
 * @param bbox_point_forPrint バウンディングボックスと直線を描画する用
 */
void CaptureBoxPoint::capture_bbox(PointSet &plypoint, PointSet &capture_point, BBox &detect_bbox, PointSet &bboxpoint_forPrint)
{

    // 面法線を求める
    auto calc_plane_normal = [](std::array<Eigen::Vector3d, 3> triangle)
    {
        Eigen::Vector3d v0v1 = triangle.at(1) - triangle.at(0);
        Eigen::Vector3d v0v2 = triangle.at(2) - triangle.at(0);

        Eigen::Vector3d plane_normal = v0v1.cross(v0v2);
        plane_normal.normalize();

        return plane_normal;
    };

    // 平面の法線と平面にある一点から 平面の方程式が出せる。
    // 係数dを求められるが、dは 平面と原点との距離である。今回の平面はすべて原点を通る平面なので、関係ない。
    auto calc_d = [](Eigen::Vector3d normal, Eigen::Vector3d point)
    {
        return (-(normal(0) * point(0) + normal(1) * point(1) + normal(2) * point(2)));
    };

    // 点を代入したときの距離が 0より上かどうかを判定する
    auto is_point_upper_side_of_plane = [](Eigen::Vector3d point, Eigen::Vector3d normal, double d)
    {
        // これは点と平面の距離
        // double flac_up = normal(0) * point(0) + normal(1) * point(1) + normal(2) * point(2) + d;
        // double flac_down = std::sqrt(std::pow(point(0), 2.0) + std::pow(point(1), 2.0) std::pow(point(2), 2.0));
        // double tmp = std::abs(flac_up) / flac_down;

        double tmp = normal(0) * point(0) + normal(1) * point(1) + normal(2) * point(2) + d;

        // std::cout
        // << "point_upper_check:" << tmp << std::endl;

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

    std::cout << "------capture_boxpoint" << std::endl;

    // 画素値から球投影の座標変換したものを格納
    std::vector<Eigen::Vector3d> box = detect_bbox.get_xyz();

    std::cout << "box:" << std::endl
              << box.at(0).transpose() << std::endl
              << box.at(1).transpose() << std::endl
              << box.at(2).transpose() << std::endl
              << box.at(3).transpose() << std::endl;

    Eigen::Vector3d origin = {0, 0, 0};

    // 4つの平面を定義するために、原点originとbboxとで三角形を作る。
    // 4角錐の各平面の面法線を求め,平面の方程式を作る。
    std::array<std::array<Eigen::Vector3d, 3>, 4> triangle_vec;
    std::array<Eigen::Vector3d, 4> normal_vec;
    std::array<double, 4> distance;

    // 四角錐定義
    // TODO: ここの取り方は逆らしいので、 外積の方向を変えてあげる。 で、is_point_upper_side_of_planeの判定を変える。
    triangle_vec.at(0) = {origin, box.at(2), box.at(0)};
    triangle_vec.at(1) = {origin, box.at(0), box.at(1)};
    triangle_vec.at(2) = {origin, box.at(3), box.at(2)};
    triangle_vec.at(3) = {origin, box.at(1), box.at(3)};

    // 四角錐の平面の面法線を計算
    // ax+by+cz+d = 0 の dを求める

    for (int i = 0; i < 4; i++)
    {
        normal_vec.at(i) = calc_plane_normal(triangle_vec.at(i));
        distance.at(i) = calc_d(normal_vec.at(i), box.at(i));
    }

    for (auto target_point : plypoint.get_point_all())
    {
        // 各平面の方程式に 点を代入し、0より大きければ、平面の上側とみなす。
        if (is_point_upper_side_of_plane(target_point, normal_vec.at(0), distance.at(0)) && is_point_upper_side_of_plane(target_point, normal_vec.at(1), distance.at(1)) && is_point_upper_side_of_plane(target_point, normal_vec.at(2), distance.at(2)) && is_point_upper_side_of_plane(target_point, normal_vec.at(3), distance.at(3)))
        {
            capture_point.add_point(target_point);
        }
    }

    std::cout
        << "plane_normal: " << normal_vec.at(0).transpose() << std::endl
        << "d: " << distance.at(0) << std::endl;
    /**
     * @brief 原点との引数の点とのedge 直線をsegpoint_with_lineに追加する
     * 原点が0番目に保存されていることが前提なので、 最初に追加しておく。
     *
     */
    auto add_edge = [&bboxpoint_forPrint](Eigen::Vector3d edge_point)
    {
        Eigen::Vector3d tmp_normalize = edge_point.normalized();

        // 極座標に変換
        double theta = std::acos(
            tmp_normalize(2) /
            std::sqrt(std::pow(tmp_normalize(0), 2.0) + std::pow(tmp_normalize(1), 2.0) + std::pow(tmp_normalize(2), 2.0)));

        double phi = std::atan2(tmp_normalize(1), tmp_normalize(0));

        // 距離rを伸ばしてpointを新たに格納
        double r = 10.0;
        Eigen::Vector3d tmp_vec = {r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta)};
        bboxpoint_forPrint.add_point(tmp_vec);

        // 原点とのedgeを格納
        long unsigned int i = 1;
        std::array<int, 2> to_zero{0, static_cast<int>(bboxpoint_forPrint.get_point_num() - i)};
        bboxpoint_forPrint.add_edge(to_zero);
    };

    //  原点とのedgeを作る用に 原点を追加
    Eigen::Vector3d zero = {0, 0, 0};
    bboxpoint_forPrint.add_point(zero);

    for (int i = 0; i < 4; i++)
    {
        add_edge(box.at(i));
    }
}
// TODO:edgeの処理をなんとか作る。

// /**
//  * @brief 原点との引数の点とのedge 直線をsegpoint_with_lineに追加する
//  * 原点が0番目に保存されていることが前提なので、 最初に追加しておく。
//  *
//  */
// auto add_edge = [&bbox_point_with_line](Eigen::Vector3d edge_point)
// {
//     Eigen::Vector3d tmp_normalize = edge_point.normalized();

//     // 極座標に変換
//     double theta = std::acos(
//         tmp_normalize(2) /
//         std::sqrt(std::pow(tmp_normalize(0), 2.0) + std::pow(tmp_normalize(1), 2.0) + std::pow(tmp_normalize(2), 2.0)));

//     double phi = std::atan2(tmp_normalize(1), tmp_normalize(0));

//     // 距離rを伸ばしてpointを新たに格納
//     double r = 20.0;
//     Eigen::Vector3d tmp_vec = {r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta)};
//     bbox_point_with_line.add_point(tmp_vec);

//     // 原点とのedgeを格納
//     long unsigned int i = 1;
//     std::array<int, 2> to_zero{0, static_cast<int>(bbox_point_with_line.get_point_num() - i)};
//     bbox_point_with_line.add_edge(to_zero);
// };

// // 原点とのedgeを作る用に 原点を追加
// Eigen::Vector3d zero = {0, 0, 0};
// bbox_point_with_line.add_point(zero);

// double allow_angle = 0.2;
// double allow_angle_radian = allow_angle * (M_PI / 180.0);
// std::cout << "allow_angle_radian:" << allow_angle_radian << std::endl;

// for (auto target_line : segmentation_point.get_point_all())
// {
//     // target_line = segmentation_point.get_point(1);
//     add_edge(target_line);
//     int region_line = check_xy_region(target_line);

//     for (auto target_point : plypoint.get_point_all())
//     {
//         if (check_xy_region(target_point) == region_line)
//         {
//             if (calc_angle_to_line(target_point, target_line) < allow_angle_radian)
//             {
//                 capture_point.add_point(target_point);
//             }
//         }
//     }
// }

/**
 * @brief 点が属す領域を返す。
 *
 * xy平面で分割した領域番号を返す。 x+y+:0, x+y-:1, x-y+:2,x-y-:3
 *
 */
auto check_xy_region = [](Eigen::Vector3d point)
{
    double x = point(0);
    double y = point(1);
    if (x > 0)
    {
        if (y > 0)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        if (y > 0)
        {
            return 2;
        }
        else
        {
            return 3;
        }
    }
};

/**
 * @brief 原点と点で結んだ直線上にある点群を抽出する。直線と点との距離で判別する。
 *
 * @param plypoint 抽出対象の点群
 * @param capture_point 抽出した点群を格納するPointSet
 * @param detect_mask 検出したマスクの点
 * @param segpoint_with_line 直線を描画する用に出力するPointSet
 */
void CaptureBoxPoint::capture_segmentation_distance(PointSet &plypoint, PointSet &capture_point, PointSet &segmentation_point, PointSet &segpoint_with_line)
{
    std::cout << "capture_segmentation_point" << std::endl;

    segmentation_point.print();

    /**
     * @brief 点と直線の距離を計算
     *
     */
    auto calc_distance_to_line = [](Eigen::Vector3d point, Eigen::Vector3d line)
    {
        Eigen::Matrix3d matrix_I = Eigen::Matrix3d::Identity();
        auto normal_line = (matrix_I - (line * line.transpose())) * point;

        // std::cout << plypoint.transpose() << std::endl
        //           << line.transpose() << std::endl
        //           << normal_line.squaredNorm() << std::endl
        //           << std::endl;

        return normal_line.squaredNorm();
    };

    /**
     * @brief 原点との引数の点とのedge 直線をsegpoint_with_lineに追加する
     * 原点が0番目に保存されていることが前提なので、 最初に追加しておく。
     *
     */
    auto add_edge = [&segpoint_with_line](Eigen::Vector3d edge_point)
    {
        Eigen::Vector3d tmp_normalize = edge_point.normalized();

        // 極座標に変換
        double theta = std::acos(
            tmp_normalize(2) /
            std::sqrt(std::pow(tmp_normalize(0), 2.0) + std::pow(tmp_normalize(1), 2.0) + std::pow(tmp_normalize(2), 2.0)));

        double phi = std::atan2(tmp_normalize(1), tmp_normalize(0));

        // 距離rを伸ばしてpointを新たに格納
        double r = 20.0;
        Eigen::Vector3d tmp_vec = {r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta)};
        segpoint_with_line.add_point(tmp_vec);

        // 原点とのedgeを格納
        long unsigned int i = 1;
        std::array<int, 2> to_zero{0, static_cast<int>(segpoint_with_line.get_point_num() - i)};
        segpoint_with_line.add_edge(to_zero);
    };

    // 原点とのedgeを作る用に 原点を追加
    Eigen::Vector3d zero = {0, 0, 0};
    segpoint_with_line.add_point(zero);

    double allow_range = 0.005;

    for (auto target_line : segmentation_point.get_point_all())
    {
        add_edge(target_line);
        int region_line = check_xy_region(target_line);

        for (auto target_point : plypoint.get_point_all())
        {
            if (check_xy_region(target_point) == region_line)
            {
                if (calc_distance_to_line(target_point, target_line) < allow_range)
                {

                    capture_point.add_point(target_point);
                }
            }
        }
    }

    std::cout << "uwaa";
}

/**
 * @brief Create histgram of captured pointcloud
 *
 */
void use_histgram(PointSet captured_point)
{
    for (auto point : captured_point.get_point_all())
    {
        std::cout << point.transpose() << std::endl;
    }
}

/**
 * @brief 原点と点で結んだ直線上にある点群を抽出する。原点とで作る直線の角度で判別する。
 *
 * @param plypoint 抽出対象の点群
 * @param capture_point 抽出した点群を格納するPointSet
 * @param detect_mask セグメンテーションで検出したmaskのpixel
 * @param detect_mask_forprint 直線を描画する用に出力するPointSet
 */
void CaptureBoxPoint::capture_segmentation_angle(PointSet &plypoint, PointSet &capture_point, Mask &detect_mask, PointSet &detect_mask_forprint)
{
    std::cout << "capture_segmentation_point" << std::endl;

    /**
     * @brief 原点と結んだ点で作る直線と角度を計算
     *
     */
    auto calc_angle_to_vector = [](Eigen::Vector3d point, Eigen::Vector3d mask)
    {
        double theta = std::acos(mask.dot(point) / point.norm());
        // double theta = std::cos(mask.dot(point) / (mask.squaredNorm() * point.squaredNorm()));

        // std::cout << "theta:" << theta << std::endl;

        // std::cout << line.dot(point) << " : " << point.squaredNorm() << " : " << line.dot(point) / (line.squaredNorm() * point.squaredNorm()) << " : " << theta << std::endl;
        return theta;
    };

    /**
     * @brief 原点との引数の点とのedge 直線をsegpoint_with_lineに追加する
     * 原点が0番目に保存されていることが前提なので、 最初に追加しておく。
     *
     */
    auto add_edge = [&detect_mask_forprint](Eigen::Vector3d edge_point)
    {
        Eigen::Vector3d tmp_normalize = edge_point.normalized();

        // 極座標に変換
        double theta = std::acos(
            tmp_normalize(2) /
            std::sqrt(std::pow(tmp_normalize(0), 2.0) + std::pow(tmp_normalize(1), 2.0) + std::pow(tmp_normalize(2), 2.0)));

        double phi = std::atan2(tmp_normalize(1), tmp_normalize(0));

        // 距離rを伸ばしてpointを新たに格納
        double r = 20.0;
        Eigen::Vector3d tmp_vec = {r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta)};
        detect_mask_forprint.add_point(tmp_vec);

        // 原点とのedgeを格納
        long unsigned int i = 1;
        std::array<int, 2> to_zero{0, static_cast<int>(detect_mask_forprint.get_point_num() - i)};
        detect_mask_forprint.add_edge(to_zero);
    };

    // 原点とのedgeを作る用に 原点を追加
    Eigen::Vector3d zero = {0, 0, 0};
    detect_mask_forprint.add_point(zero);

    // 点群の抽出
    // なす角度の許容範囲
    double allow_angle = 0.2;
    // 許容範囲をラジアンに変換
    double allow_angle_radian = allow_angle * (M_PI / 180.0);
    std::cout << "allow_angle_radian:" << allow_angle_radian << std::endl;

    // 一個一個の点に対し、抽出対象範囲の角度にある点かどうか判定する。
    for (auto mask_point : detect_mask.get_mask_xyz())
    {
        // 点の属す領域を判定して 計算を減らしてみる
        int region_line = check_xy_region(mask_point);

        for (auto target_point : plypoint.get_point_all())
        {
            if (check_xy_region(target_point) == region_line)
            {
                if (calc_angle_to_vector(target_point, mask_point) < allow_angle_radian)
                {
                    capture_point.add_point(target_point);
                    std::cout << target_point.x() << " " << target_point.y() << " " << target_point.z() << std::endl;
                }
            }
        }
        // print用に追加
        add_edge(mask_point);
    }
}
