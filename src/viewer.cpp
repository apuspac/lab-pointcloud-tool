#include "viewer.hpp"

/**
 * @brief presetに登録されたVector3d型のcolor情報を返す
 *
 * @param code 取り出すカラーコード番号。現在0-3
 * @return Eigen::Vector3d {R,G,B}のcolor
 */
Eigen::Vector3d Viewer3D::color_preset(int code)
{
    Eigen::Vector3d color;

    // switch文の回避
    // こっちの書き方のほうが プログラムできる感あって好き
    // https://gist.github.com/paosidufygthrj/035ee1900ec3022b081c
    std::map<int, std::function<void()>> color_func = {
        {0, [&color]
         { color = {0.9, 0.1, 0.1}; }},
        {1, [&color]
         { color = {0.1, 0.9, 0.1}; }},
        {2, [&color]
         { color = {0.9, 0.1, 0.9}; }},
        {3, [&color]
         { color = {0.75, 0.75, 0.75}; }},
        {4, [&color]
         { color = {0, 0, 0.9}; }}};

    color_func[code]();

    return color;
}

/**
 * @brief geometry_objに登録されたGeometryを表示する
 * これは、VisualizerWindowを使ったやり方。 機能を自分で設定する分 ちょっとめんどう。
 *
 */
void Viewer3D::show_using_custom_visualizer()
{
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow(window_name);

    for (const auto &geobj : geometry_obj)
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

/**
 * @brief geometry_objに登録されたGeometryを表示する
 *
 */
void Viewer3D::show_using_drawgeometries()
{
    open3d::visualization::DrawGeometries(get_geometry_obj(), window_name = window_name, 800, 600, 50, 50, false);
}

void Viewer3D::add_sphere()
{
    auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
    sphere->ComputeVertexNormals();
    sphere->PaintUniformColor({0.0, 1.0, 0.0});

    add_geometry_obj(sphere);
}

/**
 * @brief 座標軸を表示のために追加する
 *
 * 右手系です。
 *
 */
void Viewer3D::add_axes()
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

    add_geometry_obj(lineset);
}

/**
 * @brief 原点と各点を結んだ直線をもつGeometryのlinesetを作る
 *
 * 4角錐を作る用途で作成
 *
 * @param pointset 原点とを結ぶ点集合
 * @param color_num 色番号
 */
void Viewer3D::add_line_origin(std::vector<Eigen::Vector3d> pointset, int color_num)
{

    std::vector<Eigen::Vector3d> line_point;
    std::vector<Eigen::Vector2i> line_line;
    std::vector<Eigen::Vector3d> line_color;
    Eigen::Vector3d color = color_preset(color_num);

    // pointを追加
    for (const auto &point_xyz : pointset)
    {
        line_point.push_back(point_xyz);
    }

    // 原点と各点を結ぶlineを追加
    for (int i = 1; i < int(pointset.size()); i++)
    {
        line_line.push_back({0, i});
    }

    for (int i = 1; i < int(pointset.size()); i++)
    {
        line_color.push_back(color);
    }

    std::shared_ptr<open3d::geometry::LineSet> lineset = std::make_shared<open3d::geometry::LineSet>();
    lineset->points_ = line_point;
    lineset->lines_ = line_line;
    lineset->colors_ = line_color;

    add_geometry_obj(lineset);
}

/**
 * @brief 点群を描画用のGeometryに追加する
 *
 * @param pointset 表示する点群
 * @param color_num colorpresetで指定する色番号
 */
void Viewer3D::add_geometry_pointset(std::vector<Eigen::Vector3d> pointset, int color_num)
{
    std::shared_ptr<open3d::geometry::PointCloud> pointcloud = std::make_shared<open3d::geometry::PointCloud>();
    std::vector<Eigen::Vector3d> point_color;
    Eigen::Vector3d color = color_preset(color_num);

    // color適用はもっと良い方法ある気がするけどね。
    for (int i = 0; i < int(pointset.size()); i++)
    {
        point_color.push_back(color);
    }

    pointcloud->points_ = pointset;
    pointcloud->colors_ = point_color;

    // 法線推定
    pointcloud->EstimateNormals();

    add_geometry_obj(pointcloud);
}

// void add_sphere();
// void add_axes();
// void add_line_origin(std::vector<Eigen::Vector3d>);
// void add_geometry_pointset(std::vector<Eigen::Vector3d>);
