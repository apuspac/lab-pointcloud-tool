/**
 * @file operation.cpp
 * @brief 処理部分の実装
 */
#include "operation.hpp"

/**
 * @brief modeの値によって処理を変える関数
 *
 */
void PointOperation::mode_select()
{

    // https://yutamano.hatenablog.com/entry/2013/11/18/161234
    std::cout << "mode select" << std::endl;
    switch_func[0] = std::bind(&PointOperation::transform_rotate, this);
    switch_func[1] = std::bind(&PointOperation::transform_rotate_simulation, this);
    switch_func[2] = std::bind(&PointOperation::Rotation_only, this);
    switch_func[3] = std::bind(&PointOperation::Rotation_only_simulation, this);
    switch_func[4] = std::bind(&PointOperation::capture_boxpoint, this);
    switch_func[5] = std::bind(&PointOperation::capture_segmentation_point, this);
    switch_func[6] = std::bind(&PointOperation::capture_pointset, this);
    switch_func[9] = std::bind(&PointOperation::test_location, this);
    switch_func[get_mode()]();
}

/**
 * @brief 回転と並進を対応点から計算する
 *
 * 対応点とplyファイルを読み込む。 画像からの対応点は読み込み時に方向ベクトルに変換する。
 * 並進・回転を対応点から計算する。
 * 基本行列Eを求め、分解して tRを計算。
 * 求めた並進回転をplyファイルに適用し outputする。
 *
 * * 実行例:
 * ./Rotation   --mode 0
 *              --ply_cp plyfile.dat
 *              --img_cp img.dat
 *              --ply plyname.ply
 *              --img ../../img/kyoiku.JPG
 *              --dir ../../ply_data/transform_rotate/
 *              >! ../../ply_data/transform_rotate/out.dat
 *
 */
void PointOperation::transform_rotate()
{
    std::cout << "transform_rotate::" << std::endl;
    ObjectIO obj_io;

    // load ply対応点
    PointSet corresp_ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(0), default_dir_path, 3, corresp_ply_point);
    corresp_ply_point.print();

    // load 画像対応点 読み込む際に方向ベクトルに変換
    PointSet corresp_img_point("corresp_imgpoint");
    obj_io.load_img_point_file(corresp_img_file_name.at(0), default_dir_path, img_file_path.at(0), corresp_img_point);
    corresp_img_point.print();

    // load 元のplyファイル(回転並進させる用)
    PointSet ply_point("plyfile");
    obj_io.load_ply_point_file(ply_file_name.at(0), default_dir_path, 4, ply_point);

    CalcPointSet calc;

    // 基本行列計算
    std::vector<Eigen::Matrix<double, 9, 1>> xi_vec = calc.create_xi_vector(corresp_img_point, corresp_ply_point);

    // 行列M計算
    Eigen::Matrix<double, 9, 9> matrix_M = calc.calc_matrix_M(xi_vec);

    // 行列Mの固有値分解から基本行列Eを計算
    Eigen::Matrix3d matrix_E = calc.calc_essential_matrix(matrix_M);

    // 基本行列Eから並進t^hatを計算
    Eigen::Vector3d vector_t_diff_scale = calc.calc_translation_t(matrix_E);

    // 並進tの符号チェック
    calc.check_sign_translation_t(corresp_img_point, corresp_ply_point, vector_t_diff_scale, matrix_E);

    // 回転行列の推定
    Eigen::Matrix3d matrix_R = calc.calc_rotation_matrix_from_essential_matrix(matrix_E, vector_t_diff_scale);

    // scaleの推定
    double scale = calc.calc_scale_of_translation_t(corresp_img_point, corresp_ply_point, matrix_E, vector_t_diff_scale);

    Eigen::Vector3d translation_vector = scale * vector_t_diff_scale;

    // 理論値計算
    // Eigen::Matrix3d Rironchi;
    // Eigen::Vector3d rotate_axis = {0, 0, 0};
    // double rotate_angle = 0;
    // Rironchi = calc.calc_theory_value_Rotation_Matrix(rotate_axis, rotate_angle);

    std::cout << "----RESULT" << std::endl
              << "Essential Matrix:" << std::endl
              << matrix_E << std::endl
              << std::endl
              << "Rotation Matrix:" << std::endl
              << matrix_R << std::endl
              << std::endl
              << "scale" << std::endl
              << scale << std::endl
              << std::endl
              << "translation Vector: " << std::endl
              << translation_vector << std::endl
              << std::endl;

    // 求めた並進回転を適用させる
    ply_point.rotate(matrix_R);
    ply_point.transform(translation_vector);

    obj_io.output_ply(ply_point, default_dir_path + "match-" + ply_point.get_name() + ".ply");
}

/**
 * @brief 回転と並進を計算する(シミュレーション用)
 *
 * plyfileを読み込み、対応点をpickup, 対応点を使用して回転と並進を計算する。
 * 基本行列Eを求め、分解して tRを計算
 *
 * 読み込むファイルがどちらもplyファイルなのに注意。
 *
 * 実行例:
 * ./Rotation   --mode 1
 *              --ply_cp plyfile.dat
 *              --ply_cp img.dat
 *              --ply plyname.ply
 *              --img ../../img/kyoiku.JPG
 *              --dir ../../ply_data/transform_rotate_sim/
 *              >! ../../ply_data/transform_rotate_sim/out.dat
 *
 */
void PointOperation::transform_rotate_simulation()
{
    std::cout << "transform_rotate_simulation::" << std::endl;
    ObjectIO obj_io;

    // load plypoint
    PointSet ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(0), default_dir_path, 3, ply_point);
    ply_point.print();

    // load imgpoint(plyfile)
    PointSet img_ply_point("corresp_imgpoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(1), default_dir_path, 3, img_ply_point);
    img_ply_point.print();

    //  対応点をピックアップ
    CalcPointSet calc;
    PointSet pickup_img("pickup_img"), pickup_ply("pickup_ply");
    calc.pickup_corresp_point(img_ply_point, ply_point, pickup_img, pickup_ply, default_dir_path);
    pickup_img.print();
    pickup_ply.print();

    // img-point を 方向ベクトルへconvert
    PointSet pickup_img_convert = calc.conversion_ply_to_img_point(pickup_img);
    pickup_img_convert.print();

    // 基本行列計算
    std::vector<Eigen::Matrix<double, 9, 1>> xi_vec = calc.create_xi_vector(pickup_img_convert, pickup_ply);

    // 行列M計算
    Eigen::Matrix<double, 9, 9> matrix_M = calc.calc_matrix_M(xi_vec);

    // 行列Mの固有値分解から基本行列Eを計算
    Eigen::Matrix3d matrix_E = calc.calc_essential_matrix(matrix_M);

    // 基本行列Eから並進t^hatを計算
    Eigen::Vector3d vector_t_diff_scale = calc.calc_translation_t(matrix_E);

    pickup_img_convert.print();
    pickup_ply.print();

    // 並進tの符号チェック
    calc.check_sign_translation_t(pickup_img_convert, pickup_ply, vector_t_diff_scale, matrix_E);

    // 回転行列の推定
    Eigen::Matrix3d matrix_R = calc.calc_rotation_matrix_from_essential_matrix(matrix_E, vector_t_diff_scale);

    // scaleの推定
    double scale = calc.calc_scale_of_translation_t(pickup_img_convert, pickup_ply, matrix_R, vector_t_diff_scale);

    Eigen::Vector3d translation_vector = scale * vector_t_diff_scale;

    // 理論値計算する場合
    // Eigen::Matrix3d Rironchi;
    // Eigen::Vector3d rotate_axis = {0, 1.0, 0};
    // double rotate_angle = 30.0;
    // Rironchi = calc.calc_theory_value_Rotation_Matrix(rotate_axis, rotate_angle);

    std::cout << "----RESULT" << std::endl
              << "Essential Matrix:" << std::endl
              << matrix_E << std::endl
              << std::endl
              << "Rotation Matrix:" << std::endl
              << matrix_R << std::endl
              << std::endl
              << "scale" << std::endl
              << scale << std::endl
              << std::endl
              << "translation Vector: " << std::endl
              << translation_vector << std::endl
              << std::endl;

    // 求めた回転行列から回転軸と角度を計算する
    calc.calc_rotation_axis_from_matrix_R(matrix_R);

    // 求めた回転並進を適用
    ply_point.rotate(matrix_R);
    ply_point.transform(translation_vector);

    // 出力
    obj_io.output_ply(ply_point, default_dir_path + ply_point.get_name() + ".ply");
    obj_io.output_ply(pickup_img, default_dir_path + pickup_img.get_name() + ".ply");
    obj_io.output_ply(pickup_ply, default_dir_path + pickup_ply.get_name() + ".ply");
}

/**
 * @brief 対応点から回転のみを計算する
 *
 * 対応点から 相関行列を計算し、回転行列を計算する。
 *
 *  * 実行例:
 * ./Rotation   --mode 2
 *              --ply_cp plyfile.dat
 *              --ply_cp img.dat
 *              --ply plyname.ply
 *              --img ../../img/kyoiku.JPG
 *              --dir ../../ply_data/rotate/
 *              >! ../../ply_data/rotate/out.dat
 *
 */
void PointOperation::Rotation_only()
{
    std::cout << "Rotation only::" << std::endl;
    ObjectIO obj_io;

    // load 対応点 plypoint
    PointSet corresp_ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(0), default_dir_path, 3, corresp_ply_point);
    corresp_ply_point.print();

    // load 対応点 imgpoint
    PointSet corresp_img_point("corresp_imgpoint");
    obj_io.load_img_point_file(corresp_img_file_name.at(0), default_dir_path, img_file_path.at(0), corresp_img_point);
    corresp_img_point.print();

    // load 対象のplypoint
    PointSet ply_point("plyfile");
    obj_io.load_ply_point_file(ply_file_name.at(0), default_dir_path, 4, ply_point);

    CalcPointSet calc;

    // 重み(よくわかってない)
    double weight = 1.0;
    // 相関行列C
    Eigen::Matrix3d correlation_C = calc.calc_correlation_C(corresp_ply_point, corresp_img_point, weight);

    // 回転行列計算
    Eigen::Matrix3d matrix_R = calc.calc_rotation_matrix_from_correlation_c(correlation_C);

    // 回転行列から回転軸と角度を計算
    calc.calc_rotation_axis_from_matrix_R(matrix_R);

    // 求めた回転並進を適用
    ply_point.rotate(matrix_R);

    // 出力
    obj_io.output_ply(ply_point, default_dir_path + ply_point.get_name() + ".ply");
}

/**
 * @brief 回転を計算する(シミュレーション用)
 *
 * plyfileを読み込み、対応点をpickup, 対応点を使用して回転と並進を計算する。
 * 基本行列Eを求め、分解して tRを計算
 *
 * 読み込むファイルがどちらもplyファイルなのに注意。
 *
 * 実行例:
 * ./Rotation   --mode 3
 *              --ply_cp plyfile.dat
 *              --ply_cp img.dat
 *              --ply plyname.ply
 *              --img ../../img/kyoiku.JPG
 *              --dir ../../ply_data/transform_rotate_sim/
 *              >! ../../ply_data/transform_rotate_sim/out.dat
 *
 */
void PointOperation::Rotation_only_simulation()
{
    std::cout << "Rotation only simulation::" << std::endl;
    ObjectIO obj_io;

    // load 対応点 ply
    PointSet corresp_ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(0), default_dir_path, 3, corresp_ply_point);

    // load 対応点 img_point.ply
    PointSet corresp_imgply_point("corresp_imgpoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(1), default_dir_path, 3, corresp_imgply_point);

    //  対応点をピックアップ
    CalcPointSet calc;
    PointSet pickup_img("pickup_img"), pickup_ply("pickup_ply");
    calc.pickup_corresp_point(corresp_imgply_point, corresp_ply_point, pickup_img, pickup_ply, default_dir_path);
    pickup_img.print();
    pickup_ply.print();

    // 画像対応点を 方向ベクトルへconvert
    PointSet pickup_img_convert = calc.conversion_ply_to_img_point(pickup_img);
    pickup_img_convert.print();

    // 相関行列C
    double weight = 1.0;
    Eigen::Matrix3d correlation_C = calc.calc_correlation_C(pickup_ply, pickup_img_convert, weight);

    // 回転行列計算
    Eigen::Matrix3d matrix_R = calc.calc_rotation_matrix_from_correlation_c(correlation_C);

    // 回転行列から回転軸・角度を計算
    calc.calc_rotation_axis_from_matrix_R(matrix_R);

    // 求めた回転並進を適用
    corresp_ply_point.rotate(matrix_R);

    // 出力
    obj_io.output_ply(corresp_ply_point, default_dir_path + corresp_ply_point.get_name() + ".ply");
}

/**
 * @brief バウンディングボックスデータと原点とでできる四角錐の中にある点群を抽出する。
 *
 *
 *  * 実行例:
 * ./Rotation   --mode 4
 *              --img_cp img.dat
 *              --ply ply_data_name.ply
 *              --img ../../img/kyoiku.JPG
 *              --dir ../../ply_data/capture_bbox/
 *              >! ../../ply_data/capture_bbox/out.dat
 *
 */
void PointOperation::capture_boxpoint()
{
    // ./Rotation --ply "ply_data_name" >! ../../ply_data/"ply_data_dir"/out-"log_name".dat
    std::cout << "capture boxpoint::" << std::endl;
    ObjectIO obj_io;

    // load plydata
    PointSet ply_point("plydata");
    // obj_io.load_ply_point_file(ply_file_name.at(0), default_dir_path, 6, ply_point);
    obj_io.load_ply_point_file(ply_file_name.at(0), default_dir_path, 3, ply_point);

    // load bbox
    PointSet bbox_img_point("corresp_imgpoint");
    obj_io.load_img_point_file(corresp_img_file_name.at(0), default_dir_path, img_file_path.at(0), bbox_img_point);
    bbox_img_point.print();

    CaptureBoxPoint capbox;

    // 抽出したポイントを格納
    PointSet capture_ply("capture_bbox_point");
    // バウンディングボックス描画用
    PointSet bbox_point("bbox");
    // capbox.capture_bbox(ply_point, capture_ply, bbox_img_point, bbox_point);

    // capture_ply.print();
    obj_io.output_ply(capture_ply, default_dir_path + capture_ply.get_name() + ".ply");
    obj_io.output_ply(bbox_point, default_dir_path + bbox_point.get_name() + ".ply");
}

/**
 * @brief 検出結果の画素値から 点群を抽出する処理。
 * セグメンテーションの画素値の方向にある点群を抽出する。
 *
 * 実行例:
 * ./Rotation   --mode 5
 *              --img_cp img.dat
 *              --ply ply_data_name.ply
 *              --dir ../../ply_data/capture_bbox/
 *              >! ../../ply_data/capture_bbox/out.dat
 *
 */
void PointOperation::capture_segmentation_point()
{
    std::cout << "capture segmentation point" << std::endl;

    std::cout
        << "capture box point" << std::endl;
    ObjectIO obj_io;

    // load plypoint
    PointSet ply_point("plydata");
    // obj_io.load_ply_point_file(ply_file_name.at(0), default_dir_path, 6, ply_point);
    obj_io.load_ply_point_file(ply_file_name.at(0), default_dir_path, 3, ply_point);

    // load segmentation data
    PointSet segmentation_point("corresp_imgpoint");
    obj_io.load_img_point_file(corresp_img_file_name.at(0), default_dir_path, img_file_path.at(0), segmentation_point);
    // segmentation_point.print();

    CaptureBoxPoint capbox;

    // 抽出したポイントを格納
    PointSet capture_ply("capture_segmentation_point");
    // segmentation_lineを格納
    PointSet segline_point("segmentation_point");
    capbox.capture_segmentation_angle(ply_point, capture_ply, segmentation_point, segline_point);

    obj_io.output_ply(capture_ply, default_dir_path + capture_ply.get_name() + ".ply");
    obj_io.output_ply(segline_point, default_dir_path + segline_point.get_name() + ".ply");
}

void PointOperation::capture_pointset()
{
    std::cout << "capture boxpoint" << std::endl;
    ObjectIO obj_io;

    // load plydata
    PointSet ply_point("plydata");
    obj_io.load_ply_point_file(ply_file_name.at(0), default_dir_path, 4, ply_point);

    // load bbox ここにjsonファイルのクラスのインスタンスを宣言
    // objectIOに jsonデータ格納のプログラムを作る
    DetectionData detect;
    obj_io.load_detection_json_file(json_file_path, detect, img_file_path.at(0));

    CaptureBoxPoint capbox;

    // 抽出したポイントを格納 (とりあえず、別々に出力)
    PointSet capture_ply("capture_bbox_point");
    PointSet bbox_point("bbox");

    detect.get_bbox_data().at(0).get_bbox_all().at(0).print();

    // とりあえず 最初の1つだけ取り出す。
    BBoxData one_img_bbox = detect.get_bbox_data().at(0);
    capbox.capture_bbox(ply_point, capture_ply, one_img_bbox, bbox_point);

    std::cout << ply_point.get_point_all().at(0) << std::endl;

    Viewer3D check_ply("plyfile");
    check_ply.add_axes();
    check_ply.add_geometry_pointset(ply_point.get_point_all(), 0);

    check_ply.show_using_drawgeometries();

    // ここからsegmentation
    // 抽出したポイントを格納
    PointSet capture_ply_seg("capture_segmentation_point");
    // segmentation_lineを格納
    PointSet segline_point("segmentation_point");

    // とりあえず 最初の1つだけ取り出す。
    Mask one_img_mask = detect.get_mask_data().at(0).get_mask_data_all().at(0);
    PointSet one_mask("segmask_point");
    one_mask.add_point(one_img_mask.get_mask_xyz().at(0));

    capbox.capture_segmentation_distance(ply_point, capture_ply_seg, one_mask, segline_point);

    Viewer3D check_window("bbox");

    // ply_point: 元のply
    // capture_ply: 切り出したbboxの中の点群
    // one_img_bbox: bboxそのもの
    // bbox_point: bboxと原点とのline
    check_window.add_axes();
    check_window.add_geometry_pointset(ply_point.get_point_all(), 3);
    check_window.add_geometry_pointset(capture_ply.get_point_all(), 0);
    check_window.add_geometry_pointset(one_img_bbox.get_bbox_all().at(0).get_xyz(), 1);
    check_window.add_line_origin(bbox_point.get_point_all(), 2);

    check_window.show_using_drawgeometries();

    Viewer3D check_mask("mask");

    // plypoint:切り出し元のファイル
    // capture_ply_seg: maskで切り出したpoint
    // segline_point: maskpointと原点をつなぐline
    check_mask.add_axes();
    check_mask.add_geometry_pointset(ply_point.get_point_all(), 0);
    check_mask.add_geometry_pointset(capture_ply_seg.get_point_all(), 0);
    check_mask.add_geometry_pointset(segline_point.get_point_all(), 3);
    check_mask.add_line_origin(segline_point.get_point_all(), 1);

    check_mask.show_using_drawgeometries();

    // obj_io.output_ply(capture_ply, default_dir_path + capture_ply.get_name() + ".ply");
    // obj_io.output_ply(segline_point, default_dir_path + segline_point.get_name() + ".ply");

    // capture_ply.print();
    // obj_io.output_ply(capture_ply, default_dir_path + capture_ply.get_name() + ".ply");
    // obj_io.output_ply(bbox_point, default_dir_path + bbox_point.get_name() + ".ply");
}

/**
 * @brief テスト用の関数
 *
 * 新機能等を作ったら、あとで実装してあげる
 *
 *  * 実行例:
 * ./Rotation   --mode 9
 *              --img_cp img.dat,
 *              --ply_cp, ply.dat,
 *              --ply, plyfile.ply,
 *              --img, img/pic_point1.jpg,
 *              --dir, data/test/,
 *              --json, data/detections_test.json,
 *              --mode, 9,
 */
void pointoperation::test_location()
{
    std::cout << "test location" << std::endl;
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

    // open3d::visualization::DrawGeometries(geo);
}

/**
 * @brief 読み込むファイル名, pathをprintする
 *
 */
void PointOperation::print()
{
    std::cout << "mode:"
              << mode << std::endl;

    std::cout << std::endl
              << "default_dir_path: "
              << default_dir_path << std::endl;

    std::cout << std::endl
              << "jsonfile_path: "
              << json_file_path << std::endl;

    std::cout << std::endl
              << "corresp_img_file_name: " << std::endl;
    for (auto tmp : corresp_img_file_name)
    {
        std::cout << tmp << std::endl;
    }

    std::cout << std::endl
              << "corresp_ply_file_name: " << std::endl;
    for (auto tmp : corresp_ply_file_name)
    {
        std::cout << tmp << std::endl;
    }

    std::cout << std::endl
              << "img_file_path: " << std::endl;
    for (auto tmp : img_file_path)
    {
        std::cout << tmp << std::endl;
    }

    std::cout << std::endl
              << "ply_file_path: " << std::endl;
    for (auto tmp : ply_file_name)
    {
        std::cout << tmp << std::endl;
    }
}
