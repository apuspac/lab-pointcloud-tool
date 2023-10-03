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
    std::cout << "Point: " << name << std::endl;

    for (const auto &tmp : point3)
    {
        std::cout << std::setprecision(15) << tmp.transpose() << std::endl;
    }
    std::cout << std::endl;
    for (const auto &tmp : edge2)
    {
        std::cout << tmp.at(0) << " " << tmp.at(1) << std::endl;
    }
}

void PointSet::add_point(PointSet add_pointset)
{
    add_pointset.get_point_all();

    for (auto point : add_pointset.get_point_all())
    {
        point3.push_back(point);
    }

    // if (point3.capacity() < point3.size() + add_pointset.get_point_all().size())
    // {
    //     point3.reserve(point3.size() + add_pointset.get_point_all().size());
    // }
    // point3.insert(point3.end(), add_pointset.get_point_all().begin(), add_pointset.get_point_all().end());
    // std::cout << "oi" << std::endl;
}

/**
 * @brief pointに回転行列を適用させる
 *
 * @param rotate_matrix matrix3d
 */
void PointSet::rotate(Eigen::Matrix3d rotate_matrix)
{
    std::cout << name << " point_rotated:" << rotate_matrix << std::endl;
    for (Eigen::Vector3d &tmp : point3)
    {
        tmp = rotate_matrix * tmp;
    }
}

/**
 * @brief pointに並進を適用させる
 *
 * @param transform_vec Vector3d
 */
void PointSet::transform(Eigen::Vector3d transform_vec)
{
    std::cout << name << " point_transformed:" << transform_vec.transpose() << std::endl;
    for (Eigen::Vector3d &tmp : point3)
    {
        tmp = transform_vec + tmp;
    }
}

/**
 * @brief pointsetの重心を返す
 *
 * @return Eigen::Vector3d
 */
Eigen::Vector3d PointSet::get_center_of_gravity()
{
    std::cout << "get_center_of_gravity" << std::endl;

    Eigen::Vector3d sum = std::reduce(point3.begin(), point3.end(), Eigen::Vector3d(0, 0, 0), [](Eigen::Vector3d a, Eigen::Vector3d b)
                                      { return a + b; });
    return (sum / static_cast<double>(point3.size()));
}

void PointSet::create_histgram()
{
    std::cout << "histgram" << std::endl;

    std::vector<double> distance_from_center;

    // 中心からの距離でヒストグラムを作成する
    for (const auto &point : point3)
    {
        double tmp_dis_center = std::sqrt(std::pow(point(0), 2.0) + std::pow(point(1), 2.0));
        distance_from_center.push_back(tmp_dis_center);
    }

    std::array<int, 100> histgram_intervals = {};
    double interval = 0.05;

    auto is_within_roomrange = [](double x)
    { return (x < 100.0 && x > 0.0); };

    for (const auto &distance : distance_from_center)
    {
        if (is_within_roomrange(distance / interval))
        {
            histgram_intervals.at(static_cast<int>(distance / interval)) += 1;
        }
    }

    for (unsigned int i = 0; i < histgram_intervals.size(); i++)
    {
        std::cout << "histgram_distance_from_center" << std::endl;
        std::cout << i * interval << ", " << histgram_intervals.at(i) << std::endl;
    }

    std::array<int, 100> histgram_one_diff = {};

    // 一階差分を取ってみる
    for (std::array<int, 100>::iterator itr = histgram_intervals.begin() + 1; itr != histgram_intervals.end(); itr++)
    {
        // std::cout << *itr - *(itr - 1) << std::endl;

        // イテレータの添字は現在のitrとはじめのitrのdistanceで取れる
        histgram_one_diff.at(std::distance(histgram_intervals.begin(), itr)) = *itr - *(itr - 1);
    }

    // 一階差分で符号が反転しているところをピックアップしてみる
    auto is_extremum = [](int diff_minus1, int diff)
    {
        return (diff_minus1 * diff) < 0 && (diff_minus1 > 0);
    };

    for (std::array<int, 100>::iterator itr = histgram_one_diff.begin() + 1; itr != histgram_one_diff.end(); itr++)
    {
        if (is_extremum(*(itr - 1), *itr))
        {
            std::cout << "sign changed" << std::endl;
            std::cout << std::distance(histgram_one_diff.begin(), itr) * interval << std::endl;
        }
    }

    for (unsigned int i = 0; i < histgram_intervals.size(); i++)
    {

        // std::cout << i * interval << " " << histgram_one_diff.at(i) << std::endl;
    }
}
