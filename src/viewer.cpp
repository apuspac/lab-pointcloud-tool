// これは、VisualizerWindowを使ったやり方。 機能を自分で設定する分 ちょっとめんどう。
void Viewer3D::show_using_custom_visualizer()
{
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow(window_name);

    for (const auto geobj : geometry_obj)
    {
        vis.AddGeometry(geobj);
    }

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
}

void add_sphere()
{
    auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
    sphere->ComputeVertexNormals();
    sphere->PaintUniformColor({0.0, 1.0, 0.0});

    geometry_obj.push_back(sphere);
}

void add_axes()
{

    std::vector<Eigen::Vector3d> line_point;
    std::vector<Eigen::Vector2i> line_line;
    std::vector<Eigen::Vector3d> line_color;

    // 原点と各軸の延ばした点を使って 座標軸を表示する
    // ほかに表示する方法もあるが シンプルなので これ
    line_point.push_back({0.0, 0.0, 0});

    line_point.push_back({10.0, 0.0, 0});
    line_point.push_back({0, 10.0, 0});
    line_point.push_back({0, 0.0, 10.0});

    line_point.push_back({-10.0, 0.0, 0});
    line_point.push_back({0, -10.0, 0});
    line_point.push_back({0, 0.0, -10.0});

    // lineの定義
    // 色を変えたいので プラスマイナス別で定義
    line_line.push_back({0, 1});
    line_line.push_back({0, 2});
    line_line.push_back({0, 3});
    line_line.push_back({0, 4});
    line_line.push_back({0, 5});
    line_line.push_back({0, 6});

    line_color.push_back({0.9, 0.1, 0.1});
    line_color.push_back({0.1, 0.9, 0.1});
    line_color.push_back({0.1, 0.1, 0.9});

    line_color.push_back({0.75, 0.75, 0.75});
    line_color.push_back({0.75, 0.75, 0.75});
    line_color.push_back({0.75, 0.75, 0.75});

    std::shared_ptr<open3d::geometry::LineSet> lineset = std::make_shared<open3d::geometry::LineSet>();
    lineset->points_ = line_point;
    lineset->lines_ = line_line;
    lineset->colors_ = line_color;

    return lineset;
}