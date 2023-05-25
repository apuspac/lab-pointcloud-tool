#include "operation.hpp"
#include "pointset.hpp"
#include "calc_pointset.hpp"
#include "object_io.hpp"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <cstdio>

#include "open3d/Open3D.h"

int main(int argc, char *argv[])
{
    // ./Rotation --img_cp img.dat --ply_cp ply.dat --ply kyoiku.ply --img ../../img/kyoiku.JPG --dir ../../ply_data/test_idou/

    // 有効桁数
    std::cout << std::setprecision(15);

    PointOperation opt;
    ObjectIO::option_process(argc, argv, opt);
    opt.print();
    // opt.mode_select();

    auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
    sphere->ComputeVertexNormals();
    sphere->PaintUniformColor({0.0, 1.0, 0.0});

    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("OKOK");
    vis.AddGeometry(sphere);

    // Change view
    open3d::visualization::ViewControl &view_control = vis.GetViewControl();
    auto view_params = open3d::visualization::ViewParameters();
    view_control.ConvertToViewParameters(view_params);

    view_params.front_ = Eigen::Vector3d(0, -1, 0);
    view_params.lookat_ = Eigen::Vector3d(0, 0, 0);
    view_params.up_ = Eigen::Vector3d(0, 0, 1);
    // view_params.zoom_ = 1.0;
    view_control.ConvertFromViewParameters(view_params);

    // PollEventsはウィンドウが閉じられるときにfalseを返す
    while (vis.PollEvents() == true)
    {
        vis.UpdateGeometry();
        vis.UpdateRender();
    }

    vis.DestroyVisualizerWindow();

    std::shared_ptr<open3d::geometry::PointCloud> pointcloud = std::make_shared<open3d::geometry::PointCloud>();
    std::vector<Eigen::Vector3d> test_point, test_color;
    test_point.push_back({3.0, 2.0, 1.0});
    test_point.push_back({2.0, 1.0, 4.0});
    test_color.push_back({0.9, 0.1, 0.1});
    test_color.push_back({0.1, 0.9, 0.1});

    pointcloud->points_ = test_point;
    pointcloud->colors_ = test_color;

    // std::cout << "OK" << std::endl;
    open3d::visualization::DrawGeometries({pointcloud});

    // 終わってから 次に行く。
    open3d::visualization::DrawGeometries({sphere});

    return 0;
}