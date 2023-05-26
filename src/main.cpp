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
    opt.mode_select();

    auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
    sphere->ComputeVertexNormals();
    sphere->PaintUniformColor({0.0, 1.0, 0.0});

    std::shared_ptr<open3d::geometry::PointCloud> pointcloud = std::make_shared<open3d::geometry::PointCloud>();
    std::vector<Eigen::Vector3d> test_point, test_color;
    test_point.push_back({3.0, 2.0, 1.0});
    test_point.push_back({2.0, 1.0, 4.0});
    test_color.push_back({0.9, 0.1, 0.1});
    test_color.push_back({0.1, 0.9, 0.1});

    pointcloud->points_ = test_point;
    pointcloud->colors_ = test_color;

    // std::cout << "OK" << std::endl;
    // open3d::visualization::DrawGeometries({pointcloud});
    //

    // grid
    // auto coodinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(10000.0);
    // coodinate_frame->ComputeVertexNormals();
    // coodinate_frame->PaintUniformColor({0.0, 1.0, 0.0});

    // line_set
    // std::vector<eigen::vector3d> line_point;
    // std::vector<eigen::vector2i> line_line;
    // std::vector<eigen::vector3d> line_color;

    // line_point.push_back({0.0, 0.0, 0});
    // line_point.push_back({100.0, 0.0, 0});
    // line_point.push_back({0, 100.0, 0});
    // line_point.push_back({0, 0.0, 100.0});

    // line_line.push_back({0, 1});
    // line_line.push_back({0, 2});
    // line_line.push_back({0, 3});

    // line_color.push_back({0.9, 0.1, 0.1});
    // line_color.push_back({0.1, 0.9, 0.1});
    // line_color.push_back({0.1, 0.1, 0.9});

    // auto lineset = open3d::geometry::lineset();
    // lineset.points_ = line_point;
    // lineset.lines_ = line_line;
    // lineset.colors_ = line_color;

    // 終わってから 次に行く。
    // open3d::visualization::DrawGeometries({pointcloud, std::make_shared<open3d::geometry::LineSet>(lineset)});

    return 0;
}