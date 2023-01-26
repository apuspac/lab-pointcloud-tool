#ifndef POINTSET_HPP_INCLUDE_GUARD
#define POINTSET_HPP_INCLUDE_GUARD

#include <iostream>
#include <vector>
#include <float.h>
#include <math.h>
#include <iomanip>

#include <eigen3/Core>
#include <eigen3/LU>
#include <eigen3/Dense>

// TODO 本当はtemplateを使って Vector3d 以外も一緒に扱いたい
// けどうまくいかんかったしすぐに解決できなさそうなので後回し
// https://eigen.tuxfamily.org/dox/TopicTemplateKeyword.html
// ぱっとみ めちゃめんどそう
// template <typename T>
class PointSet
{
private:
    std::vector<Eigen::Vector3d> point3;
    std::vector<std::array<int, 2>> edge2;
    std::string name;

public:
    PointSet(std::string point_name = "none") : name(point_name) {}
    ~PointSet() {}

    void print();

    std::string get_name() { return name; }
    void add_point(Eigen::Vector3d add_point) { point3.push_back(add_point); }
    void add_edge(std::array<int, 2> edge) { edge2.push_back(edge); }

    Eigen::Vector3d get_point(uint64_t i) { return point3.at(i); }
    std::array<int, 2> get_edge(uint64_t i) { return edge2.at(i); }
    std::vector<Eigen::Vector3d> get_point_all() { return point3; }
    std::vector<std::array<int, 2>> get_edge_all() { return edge2; }

    long unsigned int get_point_num() { return point3.size(); }
    long unsigned int get_edge_num() { return edge2.size(); }

    void rotate(Eigen::Matrix3d);
    void transform(Eigen::Vector3d);
};
#endif