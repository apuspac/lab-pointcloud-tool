/**
 * @file viewer.hpp
 * @brief open3d を 扱う
 * @date 2023-05-30
 *
 */

#ifndef VIEWER_HPP_INCLUDE_GUARD
#define VIEWER_HPP_INCLUDE_GUARD

#include "open3d/Open3D.h"

class Viewer3D
{
private:
public:
    void show_sphere();
    std::shared_ptr<open3d::geometry::LineSet> show_axes();
}

#endif