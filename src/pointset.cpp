/**
 * @file pointset.cpp
 * @brief 点群を扱うpointsetの実装
 */
#include "pointset.hpp"

/**
 * @brief print
 *
 */
void PointSet::print()
{
#ifdef DEBUG
    std::cout << "Point: " << name << std::endl;

    for (const auto &tmp : point3) {
      std::cout << std::setprecision(15) << tmp.transpose() << std::endl;
    }
    std::cout << std::endl;
    for (const auto &tmp : edge2)
    {
        std::cout << tmp.at(0) << " " << tmp.at(1) << std::endl;
    }
#endif
}

void PointSet::print_polar()
{

    std::cout << "Point_polar: " << name << std::endl;

    for (const auto &tmp : point3_polar)
    {
        std::cout << std::setprecision(15) << tmp.transpose() << std::endl;
    }
}

void PointSet::add_point(PointSet add_pointset)
{
    if (add_pointset.is_empty() == false)
    {
        point3 = add_pointset.get_point_all();
    }
    if (add_pointset.is_empty_polar() == false)
    {
        point3_polar = add_pointset.get_point_all();
    }
}

void PointSet::add_point_polar(PointSet add_pointset)
{
    for (auto point : add_pointset.get_point_all())
    {
        point3_polar.push_back(point);
    }
}

/**
 * @brief pointに回転行列を適用させる
 *
 * @param rotate_matrix matrix3d
 */
void PointSet::rotate(Eigen::Matrix3d rotate_matrix)
{
    // std::cout << name << " point_rotated:" << std::endl
    //           << rotate_matrix << std::endl;
    if (is_empty() == false)
    {

        for (Eigen::Vector3d &tmp : point3)
        {
            tmp = rotate_matrix * tmp;
        }
    }
    if (is_empty_polar() == false)
    {
        convert_to_polar();

        // for (Eigen::Vector3d &tmp : point3_polar)
        // {
        //     tmp = rotate_matrix * tmp;
        // }
    }
}

/**
 * @brief pointに並進を適用させる
 *
 * @param transform_vec Vector3d
 */
void PointSet::transform(Eigen::Vector3d transform_vec)
{
    // std::cout << name << " point_transformed:" << transform_vec.transpose() << std::endl;
    if (is_empty() == false)
    {
        for (Eigen::Vector3d &tmp : point3)
        {
            tmp = tmp + transform_vec;
        }
    }

    if (is_empty_polar() == false)
    {
        convert_to_polar();
    }
}

/**
 * @brief pointsetの重心を返す
 *
 * @return Eigen::Vector3d
 */
void PointSet::calc_center_of_gravity()
{
    // std::cout << "get_center_of_gravity" << std::endl;

    Eigen::Vector3d sum = std::reduce(point3.begin(), point3.end(), Eigen::Vector3d(0, 0, 0), [](Eigen::Vector3d a, Eigen::Vector3d b)
                                      { return a + b; });
    center_of_gravity = (sum / static_cast<double>(point3.size()));
}

void PointSet::create_histgram()
{
    std::cout << "create_histgram" << std::endl;

    std::vector<double> distance_from_center;

    // 点群の中心からの距離を計算
    for (const auto &point : point3)
    {
        double tmp_dis_center = std::sqrt(std::pow(point(0), 2.0) + std::pow(point(1), 2.0));
        distance_from_center.push_back(tmp_dis_center);
    }

    // std::array<int, 500> histgram_intervals = {};
    double interval = 0.05;

    auto is_within_roomrange = [](double x)
    { return (x < 1000.0 && x > 0.0); };

    // 各点の距離をintervalで割って、histgram_intervalsに格納
    for (const auto &distance : distance_from_center)
    {
        // std::cout << (distance / interval) << "    ";
        if (is_within_roomrange(distance / interval))
        {
            histgram_intervals.at(static_cast<int>(distance / interval)) += 1;
            // std::cout << "in  " << std::endl;
        }
    }

    std::cout << "histgram_distance_from_center" << std::endl;

    // for (unsigned int i = 0; i < histgram_intervals.size(); i++)
    // {
    // std::cout << i * interval << ", " << histgram_intervals.at(i) << std::endl;
    // }

    std::array<int, 500> histgram_one_diff = {};

    // 一階差分を取ってみる
    for (std::array<int, 500>::iterator itr = histgram_intervals.begin() + 1; itr != histgram_intervals.end(); itr++)
    {
        // std::cout << *itr - *(itr - 1) << std::endl;

        // イテレータの添字は現在のitrとはじめのitrのdistanceで取れる
        histgram_one_diff.at(std::distance(histgram_intervals.begin(), itr)) = *itr - *(itr - 1);
    }

    // 一階差分で符号が反転しているところをピックアップしてみる
    // ピックアップしたピークの中から最初のピークを取得
    auto is_extremum = [](int diff_minus1, int diff)
    {
        return (diff_minus1 * diff) < 0 && (diff_minus1 > 0);
    };

    double first_peak = 0.0;

    bool flag = false;

    for (std::array<int, 500>::iterator itr = histgram_one_diff.begin() + 1; itr != histgram_one_diff.end(); itr++)
    {
        // std::cout << *(itr - 1) * (*itr) << " " << (*(itr - 1)) << std::endl;
        if (is_extremum(*(itr - 1), *itr))
        {
            // std::cout << "sign changed" << std::endl;
            // std::cout << std::distance(histgram_one_diff.begin(), itr) * interval << std::endl;
            if (flag == false)
            {
                auto peak_range = static_cast<double>(std::distance(histgram_one_diff.begin(), itr)) * interval;
                std::cout << "first peak:" << peak_range << std::endl;
                first_peak = peak_range;
                flag = true;
            }
        }
    }

    // 最初のピークを基準に 0.1m範囲の点のみを抽出
    std::vector<Eigen::Vector3d> point3_filtered;
    for (const auto &point : point3)
    {

        double tmp_dis_center = std::sqrt(std::pow(point(0), 2.0) + std::pow(point(1), 2.0));

        if (tmp_dis_center > (first_peak - 0.3) && tmp_dis_center < (first_peak + 0.3))
        {
            // std::cout << tmp_dis_center << std::endl;
            point3_filtered.push_back(point);
        }
    }

    point3 = point3_filtered;
}

void PointSet::output_hist(std::string count)
{
    std::string path = "hist-" + class_name + count + ".csv";
    std::ofstream output_data(path);

    double interval = 0.05;
    for (unsigned int i = 0; i < histgram_intervals.size(); i++)
    {
        output_data << i * interval << ", " << histgram_intervals.at(i) << std::endl;
    }

    output_data.close();
}

/**
 * @brief xyz -> r, phi, theta and save to point3_polar
 *
 */
void PointSet::convert_to_polar()
{
    // std::cout << "convert_to_polar" << std::endl;

    if(is_empty_polar()){
        point3_polar.resize(point3.size());
    }else{
        point3_polar.clear();
        point3_polar.resize(point3.size());
    }

    assert(point3_polar.size() == point3.size());

    for (auto &point : point3)
    {
        double r = std::sqrt(std::pow(point(0), 2.0) + std::pow(point(1), 2.0) + std::pow(point(2), 2.0));

        double theta = std::acos(point(2) / r);
        double phi = std::atan2(point(1), point(0));

        unsigned long index = static_cast<unsigned long>(&point - &point3[0]);

        // 最初にsizeを確保しておけば push_backじゃなくていいはずなので、 assertでチェックしておく。
        point3_polar.at(index) = Eigen::Vector3d(r, theta, phi);
    }
}

void PointSet::convert_to_rectangular()
{
    std::cout << name << " convert_to_rectangular: point_num:" << point3.size() << " " << point3_polar.size() << std::endl;
    assert(point3.size() == 0);
    // point3_polar = {r, theta, phi}
    for (auto &point : point3_polar)
    {
        double x = point(0) * sin(point(1)) * cos(point(2));
        double y = point(0) * sin(point(1)) * sin(point(2));
        double z = point(0) * cos(point(1));

        add_point(Eigen::Vector3d(x, y, z));
    }
    assert(point3_polar.size() == point3.size());
    std::cout << "convert_conpleted: " << point3.size() << " " << point3_polar.size() << std::endl;
}


/**
 * @brief 点群を高さで切り取る
 *
 * @param cutting_height 高さ方向のflag
 * @param over_flag true: 大きい点を残す false: heightより小さい点を残す、
 */
void PointSet::cutting_by_height(double cutting_height, bool over_flag)
{
    std::vector<Eigen::Vector3d> point3_polar_filtered;

    for (auto &point : point3)
    {
        if(over_flag){
            if (point(2) > cutting_height)
            {
                point3_polar_filtered.push_back(point);
            }
        }
        else{
            if (point(2) < cutting_height)
            {
                point3_polar_filtered.push_back(point);
            }
        }
    }

    point3 = point3_polar_filtered;
    convert_to_polar();

}





#ifdef OPEN3D_ENABLED

void PointSet::radius_based_filter(size_t point_num, double radius)
{
    std::cout << "radius_based_filter: radius" << radius << " num: " << point_num << std::endl;
    open3d::geometry::PointCloud pointcloud;
    std::cout << "point3.size()" << point3.size() << std::endl;
    pointcloud.points_ = point3;
    std::cout << "pointcloud.points_.size()" << pointcloud.points_.size() << std::endl;

    pointcloud.RemoveRadiusOutliers(point_num, radius, true);
    point3 = pointcloud.points_;
    std::cout << "point3.size()" << point3.size() << std::endl;
    std::cout << "pointcloud.points_.size()" << pointcloud.points_.size() << std::endl;
}


/**
 * @brief open3dのpointcloud detect floor
 *
 * @return open3d::geometry::PointCloud
 */
void DetectPlanarPatches()
{
    open3d::geometry::PointCloud pointcloud;


}




#endif
