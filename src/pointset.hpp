#ifndef POINTSET_HPP_INCLUDE_GUARD
#define POINTSET_HPP_INCLUDE_GUARD

#include <iostream>
#include <vector>
#include <eigen3/Core>

// 本当はtemplateを使って imgもplyも一緒に扱いたい けどうまくいかんかった
// template <typename T>
class PointSet
{
private:
    std::vector<Eigen::Vector3d> point3;
    std::string name;

public:
    PointSet() {}
    ~PointSet() {}

    void print();
    void add_point(Eigen::Vector3d add_point) { point3.push_back(add_point); }
    std::vector<Eigen::Vector3d> get_point() { return point3; }
    long unsigned int get_point_num() { return point3.size(); }
};

class ImagePointSet
{
};

#endif