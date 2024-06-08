/**
 * @file viewer.hpp
 * @brief open3d を 扱う
 * @date 2023-05-30
 *
 */
#ifdef OPEN3D_ENABLED
#ifndef VIEWER_HPP_INCLUDE_GUARD
#define VIEWER_HPP_INCLUDE_GUARD

#include "open3d/Open3D.h"
#include "pointset.hpp"



class Viewer3D
{
private:
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometry_obj;
    std::string window_name;

public:
    Viewer3D(std::string _name = "Open3D") : window_name(_name) {}
    ~Viewer3D() {}

    void set_window_name(std::string name) { window_name = name; }
    void add_geometry_obj(std::shared_ptr<open3d::geometry::Geometry> obj) { geometry_obj.push_back(obj); }
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> get_geometry_obj() { return geometry_obj; }

    void add_sphere();
    void add_axes();
    void add_line_origin(std::vector<Eigen::Vector3d>, int);
    void add_geometry_pointset(std::vector<Eigen::Vector3d>, int);

    void show_using_custom_visualizer();
    void show_using_drawgeometries();
    Eigen::Vector3d color_preset(int);
};
#endif
#endif
