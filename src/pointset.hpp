#ifndef POINTSET_HPP_INCLUDE_GUARD
#define POINTSET_HPP_INCLUDE_GUARD

#include <iostream>
#include <vector>
#include <eigen3/Core>

class PointSet
{
private:
    std::vector<Eigen::Vector3d> point3;
    int point_num;
    std::string name;

public:
    PointSet(std::string point_name) : name(point_name)
    {
        point_num = point3.size();
    }
    ~PointSet()
    {
        point_num = 0;
    }

    void print();
    void set_point(std::vector<Eigen::Vector3d>);
};

class ImagePointSet
{
};

#endif