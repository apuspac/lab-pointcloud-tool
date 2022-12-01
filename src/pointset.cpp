#include "pointset.hpp"

void PointSet::print()
{
    std::cout << "PointsetName: " << name << std::endl;

    for (const auto &tmp : point3)
    {
        std::cout << tmp.transpose() << std::endl;
    }
}