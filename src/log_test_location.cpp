
void ISS_point_bunny()
{
    ObjectIO obj_io;

    // load plydata
    PointSet ply_point("plydata");
    obj_io.load_ply_point_file(ply_file_name.at(0), default_dir_path, 4, ply_point);
    std::cout << default_dir_path + ply_file_name.at(0) << std::endl;

    std::shared_ptr<open3d::geometry::PointCloud> kyoiku_point = std::make_shared<open3d::geometry::PointCloud>();
    kyoiku_point->points_ = ply_point.get_point_all();

    kyoiku_point->EstimateNormals();
    kyoiku_point->PaintUniformColor({0.75, 0.75, 0.75});

    const std::string bunny_path = default_dir_path + ply_file_name.at(0);
    open3d::geometry::PointCloud bunny;
    open3d::io::ReadPointCloudOption bunny_option;

    open3d::io::ReadPointCloudFromPLY(bunny_path, bunny, bunny_option);
    bunny.EstimateNormals();
    bunny.PaintUniformColor({0.5, 0.5, 0.5});

    auto keypoints = open3d::geometry::keypoint::ComputeISSKeypoints(*kyoiku_point);
    auto keypoints_to_shape = [](std::shared_ptr<open3d::geometry::PointCloud> _keypoint)
    {
        auto spheres = open3d::geometry::TriangleMesh();

        for (const auto &keypoint : _keypoint->points_)
        {
            auto sphere = open3d::geometry::TriangleMesh::CreateSphere(0.01);
            sphere->Translate(keypoint);
            spheres += *sphere;
        }
        spheres.PaintUniformColor({1.0, 0.75, 0.0});

        return spheres;
    };

    auto keypoint_sphe = keypoints_to_shape(keypoints);

    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geo;
    std::shared_ptr<open3d::geometry::Geometry> bunny_ptr = std::make_shared<open3d::geometry::PointCloud>(bunny);
    std::shared_ptr<open3d::geometry::Geometry> keypoint_sphe_ptr = std::make_shared<open3d::geometry::TriangleMesh>(keypoint_sphe);

    Viewer3D check_window;
    check_window.add_geometry_pointset(ply_point.get_point_all(), 3);
    // check_window.add_geometry_obj(bunny_ptr);
    check_window.add_geometry_obj(keypoint_sphe_ptr);
    check_window.show_using_drawgeometries();
    // geo.push_back(bunny_ptr);
    // geo.push_back(keypoint_sphe_ptr);
}
