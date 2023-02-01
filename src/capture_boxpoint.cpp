#include "capture_boxpoint.hpp"

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
 * @brief バウンディングボックス内の点群を抽出する

 *
 * @param plypoint 抽出対象の点群
 * @param capture_point 抽出した点群を格納する
 * @param bbox_point バウンディングボックスの点
 * @param bbox_point_with_line バウンディングボックスと直線を描画する用
 */
void CaptureBoxPoint::capture_bbox(PointSet &plypoint, PointSet &capture_point, PointSet &bbox_point, PointSet &bbox_point_with_line)
{
    std::cout << "capture_boxpoint" << std::endl;

    std::cout << bbox_point_with_line.get_name() << std::endl;

    // BBOX
    std::vector<Eigen::Vector3d> box;

    // TODO: バウンディングボックスのクラスを使ってない
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
 * @param segmentation_point 原点と直線を作る点
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

    double allow_range = 0.0005;

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
 * @brief 原点と点で結んだ直線上にある点群を抽出する。原点とで作る直線の角度で判別する。
 *
 * @param plypoint 抽出対象の点群
 * @param capture_point 抽出した点群を格納するPointSet
 * @param segmentation_point 原点と直線を作る点
 * @param segpoint_with_line 直線を描画する用に出力するPointSet
 */
void CaptureBoxPoint::capture_segmentation_angle(PointSet &plypoint, PointSet &capture_point, PointSet &segmentation_point, PointSet &segpoint_with_line)
{
    std::cout << "capture_segmentation_point" << std::endl;

    segmentation_point.print();

    /**
     * @brief 原点と結んだ点で作る直線と角度を計算
     *
     */
    auto calc_angle_to_line = [](Eigen::Vector3d point, Eigen::Vector3d line)
    {
        double theta = std::acos(line.dot(point) / point.norm());

        std::cout << line.dot(point) << " : " << point.squaredNorm() << " : " << line.dot(point) / (line.squaredNorm() * point.squaredNorm()) << " : " << theta << std::endl;
        return theta;
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

    double allow_angle = 0.2;
    double allow_angle_radian = allow_angle * (M_PI / 180.0);
    std::cout << "allow_angle_radian:" << allow_angle_radian << std::endl;

    for (auto target_line : segmentation_point.get_point_all())
    {
        // target_line = segmentation_point.get_point(1);
        add_edge(target_line);
        int region_line = check_xy_region(target_line);

        for (auto target_point : plypoint.get_point_all())
        {
            if (check_xy_region(target_point) == region_line)
            {
                if (calc_angle_to_line(target_point, target_line) < allow_angle_radian)
                {
                    capture_point.add_point(target_point);
                }
            }
        }
    }
}
