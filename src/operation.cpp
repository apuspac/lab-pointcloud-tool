/**
 * @file operation.cpp
 * @brief 処理部分の実装
 */
#include "operation.hpp"
#include "matplotlibcpp.h"





/**
 * @brief modeの値によって処理を変える関数
 *
 */
void PointOperation::mode_select()
{
    // https://yutamano.hatenablog.com/entry/2013/11/18/161234
    std::cout << "mode select" << std::endl;
    switch_func[1] = std::bind(&PointOperation::transform_rotate, this);
    switch_func[2] = std::bind(&PointOperation::rotate, this);
    switch_func[3] = std::bind(&PointOperation::capture_pointset_one, this);
    switch_func[4] = std::bind(&PointOperation::old_detection_correspoint, this);
    switch_func[5] = std::bind(&PointOperation::shift_test_w_stripe_pattern, this);
    switch_func[6] = std::bind(&PointOperation::capture_point_bbox_multi, this);
    switch_func[0] = std::bind(&PointOperation::test_location, this);
    switch_func[7] = std::bind(&PointOperation::make_img_and_calc_mse, this);
    switch_func[8] = std::bind(&PointOperation::make_img_and_calc_mse_height, this);
    switch_func[get_mode()]();
    // switch_func[*] = std::bind(&PointOperation::Rotation_only_simulation, this);
    // switch_func[*] = std::bind(&PointOperation::transform_rotate_simulation, this);
    // switch_func[*] = std::bind(&PointOperation::capture_boxpoint, this);
    // switch_func[*] = std::bind(&PointOperation::capture_segmentation_point, this);
}

void PointOperation::set_date()
{
    std::string data_s = get_localtime();
    date = data_s;
}

/**
 * @brief get local time YYMMDD_HHMMSS
 *
 * @return std::string
 */
std::string PointOperation::get_localtime()
{
    std::cout << "get_date" << std::endl;
    time_t t = time(nullptr);
    const tm *local_time = localtime(&t);
    std::stringstream ss;

    // YY/MM/DD_HHMMSS
    ss << local_time->tm_year + 1900;
    ss << std::setw(2) << std::setfill('0') << local_time->tm_mon + 1;
    ss << std::setw(2) << std::setfill('0') << local_time->tm_mday;
    ss << "_";
    ss << std::setw(2) << std::setfill('0') << local_time->tm_hour;
    ss << std::setw(2) << std::setfill('0') << local_time->tm_min;
    ss << std::setw(2) << std::setfill('0') << local_time->tm_sec;

    return ss.str();
}

/**
 * @brief create out dir with date
 *
 */
void PointOperation::create_output_dir()
{
    set_date();
    std::cout << date << std::endl;
    ObjectIO::create_dir("out/" + date);
}

/**
 * @brief 読み込むファイル名, pathをprintする
 *
 */
void PointOperation::print()
{
    std::cout << "mode:" << mode << std::endl;

    if(!json_file_path.empty()){
        std::cout << "jsonfile_path: " << json_file_path << std::endl;
    }

    if(corresp_img_file_path.size() > 0){
        std::cout << std::endl
            << "corresp_img_file_name: " << std::endl;
        for (auto tmp : corresp_img_file_path)
        {
            std::cout << tmp << std::endl;
        }
    }

    if(corresp_img_file_path.size() > 0){
        std::cout << std::endl
            << "corresp_ply_file_name: " << std::endl;
        for (auto tmp : corresp_ply_file_path)
        {
            std::cout << tmp << std::endl;
        }
    }

    if(img_file_path.size() > 0){
        std::cout << std::endl
            << "img_file_path: " << std::endl;
        for (auto tmp : img_file_path)
        {
            std::cout << tmp << std::endl;
        }
    }

    if(ply_file_path.size() > 0){
        std::cout << std::endl
            << "ply_file_path: " << std::endl;
        for (auto tmp : ply_file_path)
        {
            std::cout << tmp << std::endl;
        }
    }
    if(!output_dir_path.empty()){
        std::cout << "output_dir_path: " << output_dir_path << std::endl;
    }
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
 *              >! ../../ply_data/transform_rotate/out.dat
 *
 */
void PointOperation::transform_rotate()
{
    std::cout << "transform_rotate::" << std::endl;
    ObjectIO obj_io;

    // load ply対応点
    PointSet corresp_ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_point, corresp_ply_file_path.at(0)) ;
    corresp_ply_point.print();

    // load 画像対応点 読み込む際に方向ベクトルに変換
    PointSet corresp_img_point("corresp_imgpoint");
    obj_io.load_img_point_file(img_file_path.at(0), corresp_img_point, corresp_img_file_path.at(0));
    corresp_img_point.print();

    set_date();
    obj_io.create_dir("out/" + date);

    // load 元のplyファイル(回転並進させる用)
    PointSet ply_point("plyfile");
    obj_io.load_ply_point_file(ply_point, ply_file_path.at(0) );

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

    obj_io.output_ply(ply_point, "out/" + date + "/" + "match-" + ply_point.get_name() + ".ply");
    obj_io.output_ply(corresp_ply_point, "out/" + date + "/" + "corresp-" + corresp_ply_point.get_name() + ".ply");
    obj_io.output_ply(corresp_img_point, "out/" + date + "/" + "corresp-" + corresp_img_point.get_name() + ".ply");
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
void PointOperation::rotate()
{
    std::cout << "Rotation only::" << std::endl;
    ObjectIO obj_io;

    // load 対応点 plypoint
    PointSet corresp_ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_point, corresp_ply_file_path.at(0));
    corresp_ply_point.print();

    // load 対応点 imgpoint
    PointSet corresp_img_point("corresp_imgpoint");
    obj_io.load_img_point_file(img_file_path.at(0), corresp_img_point, corresp_img_file_path.at(0));
    corresp_img_point.print();

    // load 対象のplypoint
    PointSet ply_point("plyfile");
    obj_io.load_ply_point_file(ply_point, ply_file_path.at(0));

    CalcPointSet calc;

    // 重み(よくわかってない)kamo
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
    obj_io.output_ply(ply_point, ply_point.get_name() + ".ply");
}




/**
 * @brief バウンディングボックスデータと原点とでできる四角錐の中にある点群を抽出する。
 * これは1つのBboxに対してのみ実行するテスト用。
 *
 *
 *  * 実行例:
 * ./Rotation   --ply plyfile.ply,
 *              --img img/pic_point1.png,
 *              --json data/detections_test.json,
 *              --mode *****,
 *              >! ../../ply_data/capture_bbox/out.dat
 *
 */
void PointOperation::capture_bbox_point(PointSet &ply_point_bbox, DetectionData &detect_bbox, [[maybe_unused]] Viewer3D &check_ply_bbox)
{
    std::cout << "capture boxpoint::" << std::endl;
    ObjectIO obj_io;


    CaptureDetectPoint capbox;

    // 抽出したpointを格納
    PointSet capture_ply("capture_bbox_point");
    // 可視化用
    PointSet bbox_visualization("bbox");

    // 1つのBBOXで実験
    PointSet one_capture_ply;
    PointSet one_bbox_forprint;
    BBox one_bbox = detect_bbox.get_bbox_data().at(0).get_oneIMGbbox_all().at(0);
    capbox.capture_bbox(ply_point_bbox, one_capture_ply, one_bbox, one_bbox_forprint);

    // 1つのBBOX結果を格納
    capture_ply.add_point(one_capture_ply);

    obj_io.output_ply(capture_ply, "out/" + date + "/" + capture_ply.get_name() + ".ply");

    #ifdef OPEN3D_ENABLED
    // visualiztion
    bbox_visualization.add_point(one_bbox_forprint);
    check_ply_bbox.add_geometry_pointset(ply_point_bbox.get_point_all(), 3);
    check_ply_bbox.add_geometry_pointset(capture_ply.get_point_all(), 0);
    check_ply_bbox.add_geometry_pointset(bbox_visualization.get_point_all(), 1);
    check_ply_bbox.add_line_origin(bbox_visualization.get_point_all(), 2);
    #endif

}

/**
 * @brief 検出結果の画素値から 点群を抽出する処理。
 * セグメンテーションの画素値の方向にある点群を抽出する。
 * こちらも1つのMaskに対してのみ行うテスト用
 *
 *  実行例:
 * ./Rotation   --ply plyfile.ply,
 *              --img img/pic_point1.png,
 *              --json data/detections_test.json,
 *              --mode *****,
 *              >! ../../ply_data/capture_segmentation/out.dat
 */
void PointOperation::capture_mask_point(PointSet &ply_point_mask, DetectionData &detect_mask, [[maybe_unused]] Viewer3D &check_ply_mask)
{
    std::cout << "capture segmentation point" << std::endl;
    ObjectIO obj_io;

    CaptureDetectPoint capbox;

    // 抽出したポイントを格納
    PointSet capture_ply("capture_segmentation_point");
    PointSet mask_visualization("segmentation_mask_point");

    // 一枚の画像からのMaskDataを渡す
    PointSet one_mask("one_img_segmentation_mask_point");
    PointSet one_mask_forprint("oneimg_segmentation_mask_forprint");
    Mask one_img_mask = detect_mask.get_mask_data().at(0).get_mask_data_all().at(0);

    capbox.capture_segmentation_angle(ply_point_mask, one_mask, one_img_mask, one_mask_forprint);


    // 結果を格納
    capture_ply.add_point(one_mask);
    mask_visualization.add_point(one_mask_forprint);

    obj_io.output_ply(capture_ply, "out/" + date + "/" + capture_ply.get_name() + ".ply");

#ifdef OPEN3D_ENABLED
    //  可視化用に追加
    check_ply_mask.add_geometry_pointset(capture_ply.get_point_all(), 0);
    check_ply_mask.add_geometry_pointset(mask_visualization.get_point_all(), 1);
    check_ply_mask.add_line_origin(mask_visualization.get_point_all(), 2);
#endif

}

/**
 * @brief BBOXとセグメンテーションデータの画素値に対応した点群を抽出する処理。
 * capture_bboxとcapture_segmentation_pointを組み合わせたい。
 *
 */
void PointOperation::capture_pointset_one()
{
    std::cout << "capture boxpoint" << std::endl;
    ObjectIO obj_io;

    // load plydata
    PointSet ply_point("plydata");
    obj_io.load_ply_point_file( ply_point, ply_file_path.at(0) );

    // load bbox ここにjsonファイルのクラスのインスタンスを宣言
    // objectIOに jsonデータ格納のプログラムを作る
    DetectionData detect;
    obj_io.load_detection_json_file(json_file_path, detect, img_file_path.at(0));

    detect.check_bbox_data_load();

#ifdef OPEN3D_ENABLED
    Viewer3D check_ply("plyfile");
#endif

    capture_bbox_point(ply_point, detect, check_ply);
    capture_mask_point(ply_point, detect, check_ply);


#ifdef OPEN3D_ENABLED
    check_ply.add_axes();
    check_ply.show_using_drawgeometries();
#endif
}

/**
 *  @brief segmentation maskの画素値から点群を抽出する処理のmulti
 *
 *
 *
*/
void PointOperation::test_location()
{
    std::cout << "capture segmentation point_multi" << std::endl;
    ObjectIO obj_io;

    // load plydata
    PointSet ply_point("plydata");
    obj_io.load_ply_point_file(ply_point, ply_file_path.at(0));


    // load bbox
    DetectionData detect;
    std::cout << "json_file_path: " << json_file_path << std::endl;
    obj_io.load_detection_json_file(json_file_path, detect, img_file_path.at(0));


    std::string output_dir;
    if(output_dir_path.empty())
    {
        create_output_dir();
        output_dir = "out/" + date + "/";
    }
    else{
        output_dir = output_dir_path;
    }


    // 回転角度をラジアンで指定
    // double angle = M_PI / 4.0; // 45度回転
    // double angle = 291 + 90;
    // Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d(Eigen::AngleAxisd(static_cast<double>(angle)*M_PI / 180, Eigen::Vector3d::UnitZ()));
    // ply_point.rotate(rotation_matrix);

    ply_point.transform(Eigen::Vector3d(0, 0, 0.10));


    CaptureDetectPoint capbox;

    // 抽出したポイントを格納
    PointSet capture_ply("capture_segmentation_point");
    PointSet mask_visualization("segmentation_mask_point");

    // 一枚の画像からのMaskDataを渡す
    PointSet one_mask("one_img_segmentation_mask_point");
    PointSet one_mask_forprint("oneimg_segmentation_mask_forprint");

    // Mask one_img_mask = detect_mask.get_mask_data().at(0).get_mask_data_all().at(0);
    // capbox.capture_segmentation_angle(ply_point_mask, one_mask, one_img_mask, one_mask_forprint);


    // 結果を格納
    capture_ply.add_point(one_mask);
    mask_visualization.add_point(one_mask_forprint);

    obj_io.output_ply(capture_ply, "out/" + date + "/" + capture_ply.get_name() + ".ply");


}




/**
 * @brief bboxの中にある点群を抽出する処理
 *
 *
 *  * 実行例:
 * ./Rotation   --ply, plyfile.ply,
 *              --img, img/pic_point1.png,
 *              --dir, data/test/,
 *              --json, data/detections_test.json,
 *              --mode, 6,
 */
void PointOperation::capture_point_bbox_multi()
{
    std::cout << "capture_point_bbox_multi" << std::endl;
    ObjectIO obj_io;

    // load plydata
    PointSet ply_point("plydata");
    obj_io.load_ply_point_file(ply_point, ply_file_path.at(0));


    // load bbox
    DetectionData detect;
    std::cout << "json_file_path: " << json_file_path << std::endl;
    obj_io.load_detection_json_file(json_file_path, detect, img_file_path.at(0));


    std::string output_dir;
    if(output_dir_path.empty())
    {
        create_output_dir();
        output_dir = "out/" + date + "/";
    }
    else{
        output_dir = output_dir_path;
    }


    // 回転角度をラジアンで指定
    // double angle = M_PI / 4.0; // 45度回転
    // double angle = 291 + 90;
    // Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d(Eigen::AngleAxisd(static_cast<double>(angle)*M_PI / 180, Eigen::Vector3d::UnitZ()));
    // ply_point.rotate(rotation_matrix);

    ply_point.transform(Eigen::Vector3d(0, 0, 0.10));







    // ==== bboxの中にある点群を抽出する処理
    // すべてのパーツごとの点群を格納
    std::vector<std::vector<PointSet>> all_captured_point;
    // bboxの可視化
    std::vector<std::vector<PointSet>> all_bbox_visualization;
    // 抽出したパーツの重心
    std::vector<std::vector<PointSet>> all_center_of_gravity;

    for (auto &in_img_bbox : detect.get_bbox_data())
    {

        std::vector<PointSet> captured_point_inner_bbox_multi;
        std::vector<PointSet> bbox_visualization_multi;
        std::vector<PointSet> center_of_gravity_multi;

        [[maybe_unused]]int for_histgram_output_count = 0;

        // 一つの画像の中の複数のBBoxを扱う
        for (auto &one_bbox : in_img_bbox.get_oneIMGbbox_all())
        {
            CaptureDetectPoint capbox;

            // 抽出したポイントを格納するPointSet
            PointSet captured_point_inner_bbox("captured_point_inner_bbox");
            PointSet bbox_visualization("bbox");

            capbox.capture_bbox(ply_point, captured_point_inner_bbox, one_bbox, bbox_visualization);

            if(captured_point_inner_bbox.is_empty() == false)
            {
                // captured_point_inner_bboxのヒストグラムを作成
                captured_point_inner_bbox.create_histgram();

                // 重心計算
                Eigen::Vector3d center_of_gravity = captured_point_inner_bbox.get_center_of_gravity();
                PointSet print_center_of_gravity;
                print_center_of_gravity.add_point(center_of_gravity);

                // 抽出点群 bbox 重心を格納
                captured_point_inner_bbox_multi.push_back(captured_point_inner_bbox);
                bbox_visualization_multi.push_back(bbox_visualization);
                center_of_gravity_multi.push_back(print_center_of_gravity);

#ifdef DEBUG
                std::cout << "parts_name: " << captured_point_inner_bbox.get_class_name() << std::endl;
                std::cout << "captured_point " << captured_point_inner_bbox.get_point_num() << std::endl;
#endif

                // ヒストグラム確認したいとき用
#ifdef _DEBUG
                captured_point_inner_bbox.output_hist(std::to_string(for_histgram_output_count++));
                std::cout << std::endl;
#endif
            }else{
                std::cout << "no captured point" << std::endl;

            }

        }

        all_captured_point.push_back(captured_point_inner_bbox_multi);
        all_bbox_visualization.push_back(bbox_visualization_multi);
        all_center_of_gravity.push_back(center_of_gravity_multi);

        std::cout << in_img_bbox.get_img_name() << std::endl
                  << std::endl;
    }

    //  output_ply パーツごとではなく、クラスでまとめて出力する
    std::array<PointSet, 20> output_captured_point;
    std::array<PointSet, 20> output_center_gravity;
    // PointSet output_center_gravity("output_center_gravity");

    // captured_point をまとめる処理
    for (auto &captured_point_inner_bbox : all_captured_point)
    {
        // 一枚の画像のbbox
        for (auto &captured_point : captured_point_inner_bbox)
        {
            output_center_gravity[captured_point.get_class_num()].add_point(captured_point.get_center_of_gravity());
            for (auto &point : captured_point.get_point_all())
            {
                // class_numと同じ番号のpointsetに格納
                output_captured_point[captured_point.get_class_num()].add_point(point);
                output_captured_point[captured_point.get_class_num()].set_class_num(captured_point.get_class_num());
                output_captured_point[captured_point.get_class_num()].set_class_name(captured_point.get_class_name());
            }
        }

        // center_of_gravity 重心も分けたい場合はこれも必要かも
        // for (auto &center_of_gravity_multi : all_center_of_gravity)
        // {
        //     for (auto &center_of_gravity : center_of_gravity_multi)
        //     {
        //         for (auto &point : center_of_gravity.get_point_all())
        //         {
        //             output_center_gravity.add_point(point);
        //         }
        //     }
        // }
    }

    // plyファイル出力
    for (auto &parts_point : output_captured_point)
    {
        if (parts_point.get_point_num() != 0)
        {
            obj_io.output_ply(parts_point, output_dir + parts_point.get_class_name() + "-" + std::to_string(parts_point.get_class_num()) + ".ply");
        }
    }
    // 描画処理

#ifdef OPEN3D_ENABLED
    std::cout << "check_ply_visualization" << std::endl;
    Viewer3D check_ply("check_ply");
    check_ply.add_axes();
    check_ply.add_geometry_pointset(ply_point.get_point_all(), 3);

    // all_captured_pointのPointSetをgeometrySetの中に入れる
    for (auto &captured_point_inner_bbox : all_captured_point)
    {
        for (auto &captured_point : captured_point_inner_bbox)
        {
            std::cout << captured_point.get_class_num() << std::endl;
            if (captured_point.get_point_num() != 0)
            {
                check_ply.add_geometry_pointset(captured_point.get_point_all(), captured_point.get_class_num());
                std::cout << "captured_point_vis_num: " << captured_point.get_point_num() << std::endl;
            }
        }
    }

    // bbox_visualization もlinesetと一緒に入れる
    for (auto &bbox_visualization : all_bbox_visualization)
    {
        for (auto &bbox : bbox_visualization)
        {
            // check_ply.add_geometry_pointset(bbox.get_point_all(), bbox.get_class_num());
            // check_ply.add_line_origin(bbox.get_point_all(), bbox.get_class_num());
            check_ply.add_geometry_pointset(bbox.get_point_all(), 3);
            check_ply.add_line_origin(bbox.get_point_all(), 4);
            std::cout << "bbox_vis_num" << bbox.get_point_num() << std::endl;
        }
    }

    // // center_of_gravity
    for (auto &center_of_gravity_multi : all_center_of_gravity)
    {
        for (auto &center_of_gravity : center_of_gravity_multi)
        {
            check_ply.add_geometry_pointset(center_of_gravity.get_point_all(), 4);
            check_ply.add_line_origin(center_of_gravity.get_point_all(), 4);
        }
    }

    // 描画
    check_ply.show_using_drawgeometries();
#endif

}








#ifdef OPEN3D_ENABLED
void PointOperation::projection_to_sphere()
{

    ObjectIO obj_io;
    // std::cout << "projection to sphere" << std::endl;

    // load plydata img
    PointSet ply_point("plydata");
    obj_io.load_ply_point_file(ply_point, ply_file_path.at(0) );
    InstaImg image;
    image.load_img(img_file_path.at(0));

    // convert to polar
    ply_point.convert_to_polar();
    PointSet projection_unisphere("projection_sphere");

    // 極座標変換して 距離1にすれば 単位球に投影される
    for (auto &point : ply_point.get_point_all_polar())
    {
        double phi = point(1);
        double theta = point(2);
        projection_unisphere.add_point(Eigen::Vector3d(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)));
    }

    std::cout << "check_ply_visualization" << std::endl;
    Viewer3D check_ply("check_ply");
    check_ply.add_axes();
    check_ply.add_geometry_pointset(projection_unisphere.get_point_all(), 3);

    check_ply.show_using_drawgeometries();

    // ========== ここから img 処理 ==========
    std::cout << "img edge detection" << std::endl;

    // image edge
    std::cout << image.get_name() << std::endl;

    // ガウシアンのブラーかけてノイズ除去してからCanney法
    // またopenCVに頼って... ガハハ
    EdgeImg insta_edge;
    insta_edge.detect_edge_with_canny(image.get_mat());

    // 画像のedgeの点を球の画像にプロットする
    PointSet img_projection_unisphere;
    image.convert_to_unitsphere(img_projection_unisphere);
    // img_projection_unisphere.print();

    std::cout << "check_img_visualization" << std::endl;
    Viewer3D check_img("check_img");
    check_img.add_axes();
    check_img.add_geometry_pointset(img_projection_unisphere.get_point_all(), 3);

    check_img.show_using_drawgeometries();

    // output
    obj_io.output_ply(img_projection_unisphere, "out/" + date + "/" + "img" + ".ply");
    obj_io.output_ply(projection_unisphere, "out/" +  date + "/" + "plypoint" + ".ply");
}

#endif

double img_projection(PointSet &ply_point, LidarImg &lidar_img, InstaImg &image)
{
    // 極座標から画像へ投影
    for (auto &point : ply_point.get_point_all_polar())
    {
        // thetaをxy平面からではなく、天頂角の名前のように上からの角度に変えた方がいいのかと思ったけど
        // 実質同じかもです
        // double up_theta = M_PI / 2.0 - point(1);
        // double v_dash = up_theta / M_PI;

        double u_dash = point(2) / (2.0 * M_PI);
        double v_dash = point(1) / M_PI;

        // で、おそらく画像は視点座標系で左手系になるので、
        // 右手系と合わせるために、 反転して、90度回転させる
        int u = static_cast<int>(-(u_dash * image.get_width()) + (image.get_width() / 4));
        int v = static_cast<int>(v_dash * image.get_height());

        if (u > image.get_width())
        {
            u -= static_cast<int>(image.get_width());
        }

        lidar_img.set_pixel_255(u, v);
    }

    EdgeImg LiDAR_edge;
    LiDAR_edge.detect_edge_with_sobel(lidar_img.get_mat());

    lidar_img.show("lidar", 0.25);

    // 画像を重ね合わせてみる
    image.img_alpha_blending(image.get_mat(), lidar_img.get_mat(), 0.5);

    std::cout << "ok" << std::endl;

    double mse = image.compute_MSE(image.get_mat(), lidar_img.get_mat());
    std::cout << "mse:" << mse << std::endl;
    return mse;
}

/**
 * @brief 画像シフト, 類似度mseを計算して、最も高かった場所から対応点を出力する。
 *
 *  * 実行例:
 * ./Rotation   --mode 9
 *              --img_cp img.dat,
 *              --ply_cp, ply.dat,
 *              --ply, plyfile.ply,
 *              --img, img/pic_point1.png,
 *              --dir, data/test/,
 *              --json, data/detections_test.json,
 *              --mode, 9,
 */
void PointOperation::old_detection_correspoint()
{

    ObjectIO obj_io;

    // load
    PointSet ply_point("plydata");
    obj_io.load_ply_point_file(ply_point, ply_file_path.at(0) );
    InstaImg image;
    image.load_img(img_file_path.at(0));

    // 床の点を除去
    // HACK: 本当はここも平面当てはめ PCAなどでできそうではある。
    Eigen::Vector3d floor_height = {0, 0, -1.10};

    // 点数減らしてのテストのため、 一時的に全部の点を使うように
    // Eigen::Vector3d floor_height = {0, 0, -21.0};
    PointSet removed_floor_ply_point, rmfloor_point;
    remove_pointset_floor(ply_point, removed_floor_ply_point, floor_height);
    remove_pointset_floor(ply_point, rmfloor_point, floor_height);
    // obj_io.output_ply(rmfloor_point, std::string("plypoint") + ".ply");

    // create out_dir
    set_date();
    std::cout << date << std::endl;
    obj_io.create_dir("out/" + date);

    // ========== ここから img 処理 ==========

    /**
     * @brief sobel フィルタを使ってedge検出
     *
     */
    auto insta_img_edge_detection = [this](EdgeImg &_insta_edge, InstaImg &_image)
    {
        _insta_edge.set_zero_imgMat(_image.get_height(), _image.get_width(), CV_8UC1);
        _insta_edge.detect_edge_with_sobel(_image.get_mat());
        cv::imwrite("out/" + date + "/" + "instaimg_sobel.png", _insta_edge.get_mat());
    };

    /**
     * @brief LiDAR点群を360度画像に投影してsobleフィルタによるedge検出
     *
     */
    auto lidar_to_img_edge_detection = [this](EdgeImg &_lidar_edge, PointSet &_removed_floor_ply_point, InstaImg &_image)
    {
        LidarImg lidar_img("lidar_edge");
        _removed_floor_ply_point.convert_to_polar();
        lidar_img.set_zero_imgMat(_image.get_height(), _image.get_width(), CV_8UC1);
        lidar_img.ply_to_360paranoma_img(_removed_floor_ply_point);
        cv::imwrite("out/" + date + "/" + "ply2img_origin.png", lidar_img.get_mat());

        lidar_img.closing(3, 0, 1);
        cv::imwrite("out/" + date + "/" + "ply2img_closing.png", lidar_img.get_mat());
        _lidar_edge.detect_edge_with_sobel(lidar_img.get_mat());
        cv::imwrite("out/" + date + "/" + "ply2img_closing_sobel.png", _lidar_edge.get_mat());
    };

    /**
     * @brief LiDAR点群を投影して シフトしたあと、edge検出
     *
     */
    auto lidar_to_img_edge_detection_shift = [this](EdgeImg &_lidar_edge, PointSet &_removed_floor_ply_point, InstaImg &_image, int shift_count)
    {
        LidarImg lidar_img("lidar_edge");
        LidarImg shift_tmp("shift_tmp");
        _removed_floor_ply_point.convert_to_polar();
        lidar_img.set_zero_imgMat(_image.get_height(), _image.get_width(), CV_8UC1);
        shift_tmp.set_zero_imgMat(_image.get_height(), _image.get_width(), CV_8UC1);
        lidar_img.ply_to_360paranoma_img(_removed_floor_ply_point);
        shift_tmp.set_mat(lidar_img.shift(shift_count, 0));

        shift_tmp.closing(3, 0, 1);
        _lidar_edge.detect_edge_with_sobel(shift_tmp.get_mat());
        cv::imwrite("out/" + date + "/" + "ply2img_closing_sobel.png", _lidar_edge.get_mat());
    };

    EdgeImg insta_edge("insta_edge");
    insta_img_edge_detection(insta_edge, image);

    // ========== ここから LiDAR 処理 ==========
    std::cout << std::endl
              << "LiDAR edge detection" << std::endl;
    EdgeImg lidar_edge("lidar_edge");
    lidar_to_img_edge_detection(lidar_edge, removed_floor_ply_point, image);

    EdgeImg non_shift("no_shift");
    non_shift.set_mat(lidar_edge.get_mat());

    non_shift.img_alpha_blending(non_shift.get_mat(), insta_edge.get_mat(), 1.0);
    cv::imwrite("out/" + date + "/" + "non_shift.png", non_shift.get_mat());

    std::string userInput;
    std::getline(std::cin, userInput);
    // assert(userInput == "y");

    if (userInput == "y")
    {

        std::cout << std::endl
                  << "img edge detection" << std::endl;
        std::cout << image.get_name() << std::endl;

        double eva_img = 100000;
        int count = 0;

        std::vector<std::vector<double>> eva_img_vec;

        // 原点が固定, z軸まわりの回転
        // 中心からプラスマイナス方向に動かす
        for (int i = 0; i < image.get_width() / 2; i++)
        {
            // 中心からの動かす
            EdgeImg moved_edge("moved_edge");
            moved_edge.set_mat(lidar_edge.shift(i, 0));
            double mse = insta_edge.compute_MSE(insta_edge.get_mat(), moved_edge.get_mat());
            std::cout << "i: " << i << " mse: " << mse << std::endl;
            eva_img_vec.push_back({static_cast<double>(i), mse});

            if (eva_img > mse)
            {
                eva_img = mse;
                count = i;
            }
        }

        // 反対側
        for (int i = 0; i < image.get_width() / 2; i++)
        {
            // 中心からの動かす
            EdgeImg moved_edge("moved_edge");
            moved_edge.set_mat(lidar_edge.shift(-i, 0));
            double mse = insta_edge.compute_MSE(insta_edge.get_mat(), moved_edge.get_mat());
            std::cout << "i: " << -i << " mse: " << mse << std::endl;
            eva_img_vec.push_back({static_cast<double>(-i), mse});

            if (eva_img > mse)
            {
                eva_img = mse;
                count = -i;
            }
        }

        obj_io.output_csv_2double("out/" + date + "/" + date + "evaimg.csv", eva_img_vec);
        std::cout << "RESULT : mse: " << count << " " << eva_img << std::endl;

        double shift_angle = count * 360.0 / image.get_width() - 360;
        std::cout << "angle :" << shift_angle << std::endl;

        // int count = -498;
        // double eva_img = 899.035681593153;
        int count_vertical = 0;

        std::cout << "shift_horizontal" << std::endl;
        EdgeImg shift_horizontal("shift_horizontal");
        shift_horizontal.set_mat(lidar_edge.shift(count, 0));

        shift_horizontal.img_alpha_blending(shift_horizontal.get_mat(), insta_edge.get_mat(), 1.0);
        cv::imwrite("out/" + date + "/" + "shift_horizontal.png", shift_horizontal.get_mat());

        // ====== change the height of lidar point
        // 点を動かしてから投影して、edge検出して　mse計算
        std::cout << "==============vertical_check" << std::endl;
        double split = 0.025;
        double width_search = 1.0;
        std::vector<std::vector<double>> eva_img_vec_height;

        for (int i = 0; i < static_cast<int>(width_search / split); i++)
        {
            PointSet move_point;
            rmfloor_point.transform(Eigen::Vector3d(0, 0, -split));
            move_point.add_point(rmfloor_point);
            EdgeImg lidar_edge_height_change("lidar_edge");
            lidar_to_img_edge_detection_shift(lidar_edge_height_change, move_point, image, count);

            double mse = insta_edge.compute_MSE(insta_edge.get_mat(), lidar_edge_height_change.get_mat());
            std::cout << "i: " << -i << " mse: " << mse << std::endl;
            eva_img_vec_height.push_back({static_cast<double>(-i), mse});

            if (eva_img > mse)
            {
                eva_img = mse;
                count_vertical = -i;
            }
        }

        // 初期位置に戻す
        rmfloor_point.transform(Eigen::Vector3d(0, 0, width_search));

        for (int i = 0; i < static_cast<int>(width_search / split); i++)
        {
            PointSet move_point;
            rmfloor_point.transform(Eigen::Vector3d(0, 0, split));
            move_point.add_point(rmfloor_point);
            EdgeImg lidar_edge_height_change("lidar_edge");
            lidar_to_img_edge_detection_shift(lidar_edge_height_change, move_point, image, count);

            double mse = insta_edge.compute_MSE(insta_edge.get_mat(), lidar_edge_height_change.get_mat());
            std::cout << "i: " << i << " mse: " << mse << std::endl;
            eva_img_vec_height.push_back({static_cast<double>(i), mse});

            if (eva_img > mse)
            {
                eva_img = mse;
                count_vertical = i;
            }
        }
        obj_io.output_csv_2double("out/" + date + "/" + date + "evaimg_vertical.csv", eva_img_vec_height);
        std::cout << "RESULT : mse: " << count << " " << count_vertical << " " << eva_img << std::endl;

        // 初期位置に戻す
        rmfloor_point.transform(Eigen::Vector3d(0, 0, -width_search));
    }
    // ============================ テスト環境
    // 手動でパラメータ打ち込んで テスト  mse合わせた画像を作る

    int count = -1962;
    int count_vertical = 0;
    double split = 0.025;

    // ====== RESULT Img ======
    PointSet move_point;
    rmfloor_point.transform(Eigen::Vector3d(0, 0, split * count_vertical));
    move_point.add_point(rmfloor_point);
    EdgeImg lidar_edge_height_change("lidar_change_height");
    // lidar_to_img_edge_detection_shift(lidar_edge_height_change, move_point, image, count);

    LidarImg lidar_img("lidar_edge");
    LidarImg shift_tmp("shift_tmp");
    move_point.convert_to_polar();
    lidar_img.set_zero_imgMat(image.get_height(), image.get_width(), CV_8UC1);
    shift_tmp.set_zero_imgMat(image.get_height(), image.get_width(), CV_8UC1);
    // lidar_img.resize_store_info(image.get_height(), image.get_width());
    lidar_img.resize_store_info(move_point.get_point_num());
    // lidar_img.ply_to_360paranoma_img(move_point, true);
    lidar_img.ply_to_360paranoma_img_depth(move_point, true);
    shift_tmp.set_mat(lidar_img.shift(count, 0));
    shift_tmp.closing(3, 0, 1);
    lidar_edge_height_change.detect_edge_with_sobel(shift_tmp.get_mat());
    cv::imwrite("out/" + date + "/" + "ply2img_closing_sobel.png", lidar_edge.get_mat());

    EdgeImg alpha_img("alpha_img");
    alpha_img.set_mat(lidar_edge_height_change.get_mat());
    alpha_img.img_alpha_blending(alpha_img.get_mat(), insta_edge.get_mat(), 1.0);
    cv::imwrite("out/" + date + "/" + "shift_vertical.png", alpha_img.get_mat());

    // 同じ点を取って出力。

    std::vector<Eigen::Vector3d> corres_point;
    std::vector<std::pair<int, int>> corres_img_point;

    lidar_img.get_corresponding_point_Hough(corres_point, corres_img_point, lidar_edge_height_change, insta_edge, move_point, count, date);

    obj_io.output_dat("out/" + date + "/" + date + "ply.dat", corres_point);
    obj_io.output_dat("out/" + date + "/" + date + "img.dat", corres_img_point);

    std::cout << "end more" << std::endl;
}

/**
 * @brief floor_heightの数値より高い点群を残し、他の点群を除去する. 床点群を除去したくて作成
 *
 * @param origin_point
 * @param floor_height Eigen::Vector3dで指定
 * @return PointSet&  今回戻り値が参照型なので、 戻り値をそのまま使う場合は、同じ参照で受け取る必要あり。
 */
void PointOperation::remove_pointset_floor(PointSet &origin_point, PointSet &out_point, Eigen::Vector3d floor_height)
{

    for (auto &point : origin_point.get_point_all())
    {
        if (point(2) > floor_height(2))
        {
            out_point.add_point(point);
        }
    }
}





/**
 * @brief 画像, 点群の画像を作成して、mseを計算
 *
 *
 * */
void PointOperation::make_img_and_calc_mse()
{

    std::cout << "make_img_and_calc_mse" << std::endl;
    ObjectIO obj_io;

    PointSet ply_point("plydata");
    obj_io.load_ply_point_file(ply_point, ply_file_path.at(0));
    InstaImg image;
    image.load_img(img_file_path.at(0));

    std::string output_dir;
    if(output_dir_path.empty())
    {
        create_output_dir();
        output_dir = "out/" + date + "/";
    }
    else{
        output_dir = output_dir_path;
    }

    // # make img -> theta/phi-img
    // エッジ検出
    auto insta_img_edge_detection = [this, &output_dir](EdgeImg &out_edge, InstaImg &in_img)
    {
        out_edge.set_zero_imgMat(in_img.get_height(), in_img.get_width(), CV_8UC1);
        out_edge.detect_edge_with_sobel(in_img.get_mat());
#ifdef DEBUG
        cv::imwrite(output_dir + "instaimg_sobel.png", out_edge.get_mat());
#endif
    };

    EdgeImg insta_edge("insta_edge");
    insta_img_edge_detection(insta_edge, image);
    insta_edge.make_thetaphiIMG_from_panorama( std::pair<int, int>(image.get_width(), image.get_height()));


    // # make point -> theta/phi-img
    InstaImg pointimg;



    ply_point.cutting_by_height(1.5, false);
    ply_point.cutting_by_height(-1.0,true);





    pointimg.make_thetaphiIMG_from_pointcloud(ply_point, std::pair<int, int>(image.get_width(), image.get_height()), false);
    std::cout << "make_img" << std::endl;
#ifdef DEBUG
        cv::imwrite(output_dir + "edge.png", insta_edge.get_mat());
        cv::imwrite(output_dir + "point.png", pointimg.get_mat());
#endif



    // double mse_tmp = ImgCalc::compute_MSE(pointimg.get_mat(), make_img(i, "rote" + std::to_string(i)));
    double mse_tmp = ImgCalc::compute_MSE(insta_edge.get_mat(), pointimg.get_mat());
    std::cout << "mse: " << mse_tmp << std::endl;




    // rotate start

    /**
     * 点群を回転させ、 画像を生成 cv::Matに格納し返す。
     *
     */
    auto make_img = [this, &image, &ply_point, &obj_io, &output_dir](double angle, [[maybe_unused]]std::string imgname)
    {
        // pointcloud rotate
        PointSet ply_point_rotate("rotate_point");

        ply_point_rotate.add_point(ply_point);
        ply_point_rotate.rotate(Eigen::Matrix3d(Eigen::AngleAxisd(angle*M_PI / 180, Eigen::Vector3d::UnitZ())));

        InstaImg pointimg_lidar;
        pointimg_lidar.make_thetaphiIMG_from_pointcloud(ply_point_rotate, std::pair<int, int>(image.get_width(), image.get_height()), true);

#ifdef _DEBUG
        cv::imwrite(output_dir + imgname + ".png", pointimg_lidar.get_mat());
        obj_io.output_ply(ply_point_rotate, output_dir + imgname + ".ply");
#endif

        return pointimg_lidar.get_mat();
    };

    std::cout << "MSE calc" << std::endl;
    std::vector<double> mse_vec, mse_vec_ave;

    // 0.5度ずつ回転させて、mseを計算
    for (double i = 0; i <= 360; i+=0.5)
    {
        double calc_mse = ImgCalc::compute_MSE(insta_edge.get_mat(), make_img(i, "rote" + std::to_string(i)));
        mse_vec.push_back(calc_mse);
#ifdef DEBUG
        std::cout << "angle: " << i << " mse: " << calc_mse<< std::endl;
#endif
    }

    obj_io.output_dat(output_dir +  "mse.dat", mse_vec);

    std::vector<double> mse_result;
    std::vector<double>::iterator minIt = std::min_element(mse_vec.begin(), mse_vec.end());
    // *minItで最小値
    long minIndex = std::distance(mse_vec.begin(), minIt);

    std::cout << "min:" << *minIt << "minIndex: " << minIndex << std::endl;
    mse_result.push_back(*minIt);
    mse_result.push_back(static_cast<double>(minIndex));
    obj_io.output_dat(output_dir_path + "mse.dat", mse_vec);
    obj_io.output_dat(output_dir_path + "mse_result.dat", mse_result);


    // pointcloud rotate
    PointSet ply_point_calc_rotate("rotate_point");
    ply_point_calc_rotate.add_point(ply_point);
    // NOTE:90°プラスしているのは これするとつじつまが合うからですが、なんで90°ずれるのか原因が分かっていない。
    // -> 座標系がうんたらかんたら。
    ply_point_calc_rotate.rotate(Eigen::Matrix3d(Eigen::AngleAxisd(static_cast<double>(minIndex + 90)*M_PI / 180, Eigen::Vector3d::UnitZ())));
    // ply_point_calc_rotate.rotate(Eigen::Matrix3d(Eigen::AngleAxisd(24.0*M_PI / 180, Eigen::Vector3d::UnitZ())));
    InstaImg pointimg_lidar_mse;
    pointimg_lidar_mse.make_thetaphiIMG_from_pointcloud(ply_point_calc_rotate, std::pair<int, int>(image.get_width(), image.get_height()), true);



    cv::imwrite(output_dir_path + "mse_lidar" + ".png", pointimg_lidar_mse.get_mat());

    // ply_point_calc_rotate.rotate(Eigen::Matrix3d(Eigen::AngleAxisd(static_cast<double>(minIndex)*M_PI / 180, Eigen::Vector3d::UnitZ())));

    ply_point.rotate(Eigen::Matrix3d(Eigen::AngleAxisd(static_cast<double>(minIndex+90)*M_PI / 180, Eigen::Vector3d::UnitZ())));


    obj_io.output_ply(ply_point, output_dir + "mse_lidar"+ ".ply");

#ifdef MATPLOTLIB_ENABLED
    plt::plot(mse_vec);
    plt::save(output_dir + date + "mse.png");
    plt::show();
#endif


}

void PointOperation::make_img_and_calc_mse_height()
{
    std::cout << "make_img_and_calc_mse_height" << std::endl;
    ObjectIO obj_io;

    PointSet ply_point("plydata");
    obj_io.load_ply_point_file(ply_point, ply_file_path.at(0));
    InstaImg image;
    image.load_img(img_file_path.at(0));

    std::string output_dir;
    if(output_dir_path.empty())
    {
        create_output_dir();
        output_dir = "out/" + date + "/";
    }
    else{
        output_dir = output_dir_path;
    }

    // # make img -> theta/phi-img
    // エッジ検出
    auto insta_img_edge_detection = [this, &output_dir](EdgeImg &out_edge, InstaImg &in_img)
    {
        out_edge.set_zero_imgMat(in_img.get_height(), in_img.get_width(), CV_8UC1);
        out_edge.detect_edge_with_sobel(in_img.get_mat());
#ifdef DEBUG
        cv::imwrite(output_dir + "instaimg_sobel.png", out_edge.get_mat());
#endif
    };

    EdgeImg insta_edge("insta_edge");
    insta_img_edge_detection(insta_edge, image);
    insta_edge.make_thetaphiIMG_from_panorama( std::pair<int, int>(image.get_width(), image.get_height()));

    InstaImg pointimg;

    // # make point -> theta/phi-img
    ply_point.cutting_by_height(1.5, false);
    ply_point.cutting_by_height(-1.0,true);

    //TODO,NOTE: minIndexは 実行した結果を使っている。
    //index:48, mse:577.3853
    long minIndex = 48;
    ply_point.rotate(Eigen::Matrix3d(Eigen::AngleAxisd(static_cast<double>(minIndex+90)*M_PI / 180, Eigen::Vector3d::UnitZ())));


    pointimg.make_thetaphiIMG_from_pointcloud(ply_point, std::pair<int, int>(image.get_width(), image.get_height()), false);
    
    std::cout << "make_img" << std::endl;
#ifdef DEBUG
        cv::imwrite(output_dir + "edge.png", insta_edge.get_mat());
        cv::imwrite(output_dir + "point.png", pointimg.get_mat());
#endif


    bool end_frag = false;
    assert(end_frag == false);

    // --- height
    auto make_img_height = [this, &image, &ply_point, &obj_io, &output_dir](double height, [[maybe_unused]]std::string imgname)
    {
        // pointcloud rotate
        PointSet ply_point_height("rotate_point");

        ply_point_height.add_point(ply_point);
        ply_point_height.transform(Eigen::Vector3d(0, 0, height));

        std::cout << ply_point_height.get_point(0) << std::endl;

        InstaImg pointimg_lidar;

        pointimg_lidar.make_thetaphiIMG_from_pointcloud(ply_point_height, std::pair<int, int>(image.get_width(), image.get_height()), true);

#ifdef DEBUG
        cv::imwrite(output_dir + imgname + "_height.png", pointimg_lidar.get_mat());
        // obj_io.output_ply(ply_point_height, output_dir + imgname + "_height.ply");
#endif
        return pointimg_lidar.get_mat();

    };


    std::cout << ">>>>> MSE height calc" << std::endl;
    std::vector<double> mse_vec_height, mse_vec_ave_height;



    // 0.5cmずつ移動
    for (double i = -0.15; i <= 0.15; i+=0.005)
    {
        double calc_mse_height = ImgCalc::compute_MSE(insta_edge.get_mat(), make_img_height(i, "height" + std::to_string(i)));
        mse_vec_height.push_back(calc_mse_height);
#ifdef DEBUG
        std::cout << "height: " << i << " mse: " << calc_mse_height<< std::endl;
#endif
    }


    obj_io.output_dat(output_dir +  "mse_height.dat", mse_vec_height);

    std::vector<double> mse_result_height;
    std::vector<double>::iterator minIt_height = std::min_element(mse_vec_height.begin(), mse_vec_height.end());
    // *minItで最小値
    long minIndex_height = std::distance(mse_vec_height.begin(), minIt_height);

    std::cout << "min:" << *minIt_height << "minIndex: " << minIndex_height << std::endl;
    mse_result_height.push_back(*minIt_height);
    mse_result_height.push_back(static_cast<double>(minIndex_height));
    obj_io.output_dat(output_dir_path + "mse_height.dat", mse_vec_height);
    obj_io.output_dat(output_dir_path + "mse_result_height.dat", mse_result_height);


    // pointcloud translate
    PointSet ply_point_calc_height("translate_point");
    ply_point_calc_height.add_point(ply_point);
    ply_point_calc_height.transform(Eigen::Vector3d(0, 0, static_cast<double>(minIndex_height) * 0.5 + -15.0));
    ply_point_calc_height.rotate(Eigen::Matrix3d(Eigen::AngleAxisd(static_cast<double>(minIndex+90)*M_PI / 180, Eigen::Vector3d::UnitZ())));


    InstaImg pointimg_lidar_mse_height;
    pointimg_lidar_mse_height.make_thetaphiIMG_from_pointcloud(ply_point_calc_height, std::pair<int, int>(image.get_width(), image.get_height()), true);

    cv::imwrite(output_dir_path + "mse_lidar_height" + ".png", pointimg_lidar_mse_height.get_mat());

    // ply_point.rotate(Eigen::Matrix3d(Eigen::AngleAxisd(static_cast<double>(minIndex+90)*M_PI / 180, Eigen::Vector3d::UnitZ())));


    obj_io.output_ply(ply_point_calc_height, output_dir + "mse_lidar_height"+ ".ply");

#ifdef MATPLOTLIB_ENABLED
    plt::plot(mse_vec_height);
    plt::save(output_dir + date + "mse_height.png");
    plt::show();
#endif
}


// peakを見るために、一階差分の配列(size-1)を返す
template<class T>
std::vector<T> calculate_diff(std::vector<T> &mse_vec_lambda)
{
    std::vector<T> mse_diff_lambda;
    for(auto itr = mse_vec_lambda.begin(); itr != mse_vec_lambda.end(); ++itr)
    {
        mse_diff_lambda.push_back(*itr - *(itr - 1));
    }
    return mse_diff_lambda;
}


// 一階差分との符号を確認して極値を返す。このとき、差がthreshold以上のものを持ってくる
template<class T>
std::vector<bool> find_local_max(std::vector<T> &mse_diff_lambda, double threshold)
{

    // size()で、falseで初期化。
    std::vector<bool> mse_peak_lambda(mse_diff_lambda.size(), false);

    for (auto itr = mse_diff_lambda.begin(); itr != mse_diff_lambda.end(); ++itr)
    {
        if (*itr > 0 && *itr * (*(itr - 1)) < 0 && *itr > threshold)
        {
            mse_peak_lambda[itr- mse_diff_lambda.begin()] = true;
        }
    }
    return mse_peak_lambda;

}




/**
 * 画像シフトのテストを作って 試します。
 *
 */
void PointOperation::shift_test_w_stripe_pattern()
{
    std::cout << "" << std::endl;
    ObjectIO obj_io;

    PointSet ply_point("plydata");
    PointSet watermelon_point("watermelon_point");
    PointSet stitch_edge_point("stitch_edge_point");

    std::vector<int> stitch_edge;

    stitch_edge.resize(360 * 180);

    create_output_dir();

    int divide = 5;
    CalcPointSet::make_striped_pattern(stitch_edge, stitch_edge_point, watermelon_point, divide);

    watermelon_point.convert_to_rectangular();
    obj_io.output_ply(watermelon_point, "out/" + date + "/" + date + ".ply");
    stitch_edge_point.convert_to_rectangular();
    obj_io.output_ply(stitch_edge_point, "out/" + date + "/_" + date + ".ply");

    // #### make 球-> 画像
    InstaImg pointimg;
    pointimg.make_thetaphiIMG_from_pointcloud(stitch_edge_point, std::pair<int, int>(360, 180));

    cv::imwrite("out/" + date + "/" + "insta.png", pointimg.get_mat());


    /**
     * 点群を回転させ、 画像を生成 cv::Matに格納し返す。
     *
     */
    auto make_img = [this, &stitch_edge_point, &obj_io](double angle, [[maybe_unused]]std::string imgname)
    {
        // pointcloud rotate
        PointSet stitch_edge_point_rotate("stitch_edge_point");
        stitch_edge_point_rotate.add_point(stitch_edge_point);
        stitch_edge_point_rotate.rotate(Eigen::Matrix3d(Eigen::AngleAxisd(angle*M_PI / 180, Eigen::Vector3d::UnitZ())));
        InstaImg pointimg_lidar;
        pointimg_lidar.make_thetaphiIMG_from_pointcloud(stitch_edge_point_rotate, std::pair<int, int>(360, 180));

#ifdef _DEBUG
        cv::imwrite("out/" + date + "/" + imgname + ".png", pointimg_lidar.get_mat());
        obj_io.output_ply(stitch_edge_point_rotate, "out/" + date + "/_" + imgname + ".ply");
#endif

        return pointimg_lidar.get_mat();
    };

    std::cout << "MSE calc" << std::endl;
    std::vector<double> mse_vec, mse_vec_ave;


    // 360°回転させ、mseを計算
    for (double i = 0; i <= 360; i++)
    {
        double mse_tmp = ImgCalc::compute_MSE(pointimg.get_mat(), make_img(i, "rote" + std::to_string(i)));
        mse_vec.push_back(mse_tmp);

    }

    obj_io.output_dat("out/" + date + "/" + date + "mse.dat", mse_vec);


#ifdef MATPLOTLIB_ENABLED
    plt::plot(mse_vec);
#endif




    // plot, dat graph化の処理
    std::vector<double> mse_diff = calculate_diff(mse_vec);
    std::vector<bool> mse_peak = find_local_max(mse_diff, 500);

    std::vector<int> plot_mse;
    std::vector<double> plot_dat;
    std::vector<int> plot_true;

    int interval = 360 / divide;
    for(int i = 0; i < static_cast<int>(mse_vec.size());  i += interval)
    {
        plot_true.push_back(i);
    }



    for(int i = 0; i < static_cast<int>(mse_peak.size()); i++)
    {
        std::cout << i << " " << mse_peak.at(i) << std::endl;
        if(mse_peak.at(i)){
            plot_mse.push_back(100);
            plot_dat.push_back(i);
        }
        else{
            plot_mse.push_back(0);
        }
    }

    obj_io.output_dat("out/" + date + "/" + date + "peak.dat", plot_dat);
    obj_io.output_dat("out/" + date + "/" + date + "peak_true.dat", plot_true);


#ifdef MATPLOTLIB_ENABLED
    plt::plot(plot_mse);
    plt::show();
#endif

}



/**
 * @brief 回転と並進を計算する(シミュレーション用)
 *
 * plyfileを読み込み、対応点をpickup, 対応点を使用して回転と並進を計算する。
 * 基本行列Eを求め、分解して tRを計算
 *
 * 読み込むファイルがどちらもplyファイルなのに注意。
 * 回転と姿勢だけが違うplyファイルを読み込んで、 原点合わせをする。
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

    set_date();
    obj_io.create_dir("out/" + date);

    // load plypoint
    PointSet ply_point("corresp_plypoint");
    obj_io.load_ply_point_file( ply_point, corresp_ply_file_path.at(0) );
    ply_point.print();

    // load imgpoint(plyfile)
    PointSet img_ply_point("corresp_imgpoint");
    obj_io.load_ply_point_file(img_ply_point, corresp_ply_file_path.at(1) );
    img_ply_point.print();

    //  対応点をピックアップ
    CalcPointSet calc;
    PointSet pickup_img("pickup_img"), pickup_ply("pickup_ply");
    calc.pickup_corresp_point(img_ply_point, ply_point, pickup_img, pickup_ply);
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
    obj_io.output_ply(ply_point, "out/" + date + ply_point.get_name() + ".ply");
    obj_io.output_ply(pickup_img, "out/" + date + pickup_img.get_name() + ".ply");
    obj_io.output_ply(pickup_ply, "out/" + date + pickup_ply.get_name() + ".ply");
}

/**
 * @brief 回転を計算する(シミュレーション用)
 *
 * 生成した 回転のみが変わっているplyファイルを読み込み、
 * 対応点をpickup, 回転と並進を計算する。
 * 対応を得る際に pointの番号で指定しているため 完全なコピーのplyファイルじゃないとうまくいかない。
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
void PointOperation::rotate_simulation()
{
    std::cout << "Rotation only simulation::" << std::endl;
    ObjectIO obj_io;

    // load 対応点 ply
    PointSet corresp_ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_point, corresp_ply_file_path.at(0) );

    // load 対応点 img_point.ply
    PointSet corresp_imgply_point("corresp_imgpoint");
    obj_io.load_ply_point_file(corresp_imgply_point, corresp_ply_file_path.at(1) );

    //  対応点をピックアップ
    CalcPointSet calc;
    PointSet pickup_img("pickup_img"), pickup_ply("pickup_ply");
    calc.pickup_corresp_point(corresp_imgply_point, corresp_ply_point, pickup_img, pickup_ply);
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
    obj_io.output_ply(corresp_ply_point, corresp_ply_point.get_name() + ".ply");
}
