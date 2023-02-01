/**
 * @file pointset.hpp
 * @brief 点群・plyファイルを扱う
 */
#ifndef POINTSET_HPP_INCLUDE_GUARD
#define POINTSET_HPP_INCLUDE_GUARD

#include <iostream>
#include <vector>
#include <float.h>
#include <math.h>
#include <iomanip>
#include <unordered_map>

#include <eigen3/Core>
#include <eigen3/LU>
#include <eigen3/Dense>

/**
 * @brief 点群そのものに関するクラス
 *
 */
class PointSet
{
private:
    // 点群を扱う。 点単体は Eigen::Vector3
    std::vector<Eigen::Vector3d> point3;

    std::vector<std::array<int, 2>> edge2;
    std::string name;

public:
    PointSet(std::string point_name = "none") : name(point_name) {}
    ~PointSet() {}

    void print();

    std::string get_name() { return name; }

    // 点を最後に追加
    void add_point(Eigen::Vector3d add_point) { point3.push_back(add_point); }
    // edgeの組を追加
    void add_edge(std::array<int, 2> edge) { edge2.push_back(edge); }

    /**
     * @brief Get the point object
     * 単体が欲しいとき
     * これget_point_all()で渡して そっちで.at()したほうがいいかも？
     *
     * @param i 欲しい点の番号
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d get_point(uint64_t i) { return point3.at(i); }
    std::array<int, 2> get_edge(uint64_t i) { return edge2.at(i); }

    // 全体getter
    std::vector<Eigen::Vector3d> get_point_all() { return point3; }
    std::vector<std::array<int, 2>> get_edge_all() { return edge2; }

    // point3の総数を返す
    long unsigned int get_point_num() { return point3.size(); }
    long unsigned int get_edge_num() { return edge2.size(); }

    void rotate(Eigen::Matrix3d);
    void transform(Eigen::Vector3d);
};
#endif

// 本当はtemplateを使って Vector3d 以外も一緒に扱えるようにしてみたかった
// けどうまくいかんかったしすぐに解決できなさそうなので後回し
// ぱっとみ めちゃめんどそう
// https://eigen.tuxfamily.org/dox/TopicTemplateKeyword.html
