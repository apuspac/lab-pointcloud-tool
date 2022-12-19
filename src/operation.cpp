#include "operation.hpp"

/**
 * @brief
 *
 */
void PointOperation::transform_rotate()
{
    std::cout << "transform_rotate" << std::endl;
    ObjectIO obj_io;

    // load
    PointSet corresp_ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(0), default_dir_path, 3, corresp_ply_point);
    corresp_ply_point.print();

    PointSet corresp_img_point("corresp_imgpoint");
    obj_io.load_img_point_file(corresp_img_file_name.at(0), default_dir_path, img_file_path.at(0), corresp_img_point);
    corresp_img_point.print();

    PointSet ply_point("plyfile");
    obj_io.load_ply_point_file(ply_file_name.at(0), default_dir_path, 4, ply_point);

    CalcPointSet calc;

    // 理論値計算
    Eigen::Matrix3d Rironchi;
    Eigen::Vector3d rotate_axis = {0, 0, 0};
    double rotate_angle = 0;
    Rironchi = calc.calc_theory_value_Rotation_Matrix(rotate_axis, rotate_angle);

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

    obj_io.output_ply(corresp_ply_point, default_dir_path + corresp_ply_point.get_name() + ".ply");
}

void PointOperation::transform_rotate_simulation()
{
    // ./Rotation --ply_cp r0_0_1-d30.dat --ply_cp img.dat --dir ../../ply_data/check_1130/
    // ./Rotation --ply_cp t0_0_5.0.dat --ply_cp img.dat --dir ../../ply_data/check_1130/
    std::cout << "transform_rotate_simulation::" << std::endl;
    ObjectIO obj_io;

    // load
    PointSet corresp_ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(0), default_dir_path, 3, corresp_ply_point);
    corresp_ply_point.print();

    // PointSet corresp_img_point("corresp_imgpoint");
    // obj_io.load_img_point_file(corresp_img_file_name.at(0), default_dir_path, img_file_path.at(0), corresp_img_point);
    // corresp_img_point.print();
    PointSet corresp_imgply_point("corresp_imgpoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(1), default_dir_path, 3, corresp_imgply_point);
    corresp_imgply_point.print();

    //  対応点をピックアップ
    CalcPointSet calc;
    PointSet pickup_img("pickup_img"), pickup_ply("pickup_ply");
    calc.pickup_corresp_point(corresp_imgply_point, corresp_ply_point, pickup_img, pickup_ply);
    pickup_img.print();
    pickup_ply.print();

    // 方向ベクトルへconvert
    PointSet pickup_img_convert = calc.conversion_ply_to_img_point(pickup_img);
    pickup_img_convert.print();
    // obj_io.output_ply(pickup_img_convert, default_dir_path + "pickup_img_convert" + ".ply");

    // 理論値計算
    Eigen::Matrix3d Rironchi;
    Eigen::Vector3d rotate_axis = {0, 1.0, 0};
    double rotate_angle = 30.0;
    Rironchi = calc.calc_theory_value_Rotation_Matrix(rotate_axis, rotate_angle);

    // 基本行列計算
    std::vector<Eigen::Matrix<double, 9, 1>> xi_vec = calc.create_xi_vector(pickup_img_convert, pickup_ply);

    // 行列M計算
    Eigen::Matrix<double, 9, 9> matrix_M = calc.calc_matrix_M(xi_vec);

    // 行列Mの固有値分解から基本行列Eを計算
    Eigen::Matrix3d matrix_E = calc.calc_essential_matrix(matrix_M);

    // 基本行列Eから並進t^hatを計算
    Eigen::Vector3d vector_t_diff_scale = calc.calc_translation_t(matrix_E);

    // 並進tの符号チェック
    calc.check_sign_translation_t(pickup_img_convert, pickup_ply, vector_t_diff_scale, matrix_E);

    // 回転行列の推定
    Eigen::Matrix3d matrix_R = calc.calc_rotation_matrix_from_essential_matrix(matrix_E, vector_t_diff_scale);

    // scaleの推定
    double scale = calc.calc_scale_of_translation_t(pickup_img_convert, pickup_ply, matrix_R, vector_t_diff_scale);
    // double scale = calc.calc_scale_of_translation_t(pickup_img_convert, pickup_ply, matrix_R, tmp);

    Eigen::Vector3d translation_vector = scale * vector_t_diff_scale;

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

    calc.calc_rotation_axis_from_matrix_R(matrix_R);

    corresp_ply_point.rotate(matrix_R);
    corresp_ply_point.transform(translation_vector);

    obj_io.output_ply(corresp_ply_point, default_dir_path + pickup_ply.get_name() + ".ply");

    // obj_io.output_ply(corresp_ply_point, default_dir_path + corresp_ply_point.get_name() + ".ply");
}

void PointOperation::Rotation_point()
{
    std::cout << "Rotation Point" << std::endl;
    ObjectIO obj_io;

    // load
    PointSet corresp_ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(0), default_dir_path, 3, corresp_ply_point);
    corresp_ply_point.print();

    PointSet corresp_img_point("corresp_imgpoint");
    obj_io.load_img_point_file(corresp_img_file_name.at(0), default_dir_path, img_file_path.at(0), corresp_img_point);
    corresp_img_point.print();

    PointSet ply_point("plyfile");
    obj_io.load_ply_point_file(ply_file_name.at(0), default_dir_path, 4, ply_point);

    CalcPointSet calc;
    // 相関行列C
    double weight = 1.0;
    Eigen::Matrix3d correlation_C = calc.calc_correlation_C(corresp_ply_point, corresp_img_point, weight);

    // 回転行列計算
    Eigen::Matrix3d rotation_matrix_R = calc.calc_rotation_matrix_from_correlation_c(correlation_C);

    calc.calc_rotation_axis_from_matrix_R(rotation_matrix_R);
    obj_io.output_ply(corresp_ply_point, default_dir_path + corresp_ply_point.get_name() + ".ply");
}

void PointOperation::Rotation_point_simulation()
{
    // ./Rotation --ply_cp r0_0_1-d30.dat --ply_cp img.dat --dir ../../ply_data/check_1130/
    // ./Rotation --ply_cp t0_0_5.0.dat --ply_cp img.dat --dir ../../ply_data/check_1130/

    std::cout << "Rotation Point" << std::endl;
    ObjectIO obj_io;

    // load
    PointSet corresp_ply_point("corresp_plypoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(0), default_dir_path, 3, corresp_ply_point);
    corresp_ply_point.print();

    PointSet corresp_imgply_point("corresp_imgpoint");
    obj_io.load_ply_point_file(corresp_ply_file_name.at(1), default_dir_path, 3, corresp_imgply_point);
    corresp_imgply_point.print();

    //  対応点をピックアップ
    CalcPointSet calc;
    PointSet pickup_img("pickup_img"), pickup_ply("pickup_ply");
    calc.pickup_corresp_point(corresp_imgply_point, corresp_ply_point, pickup_img, pickup_ply);
    pickup_img.print();
    pickup_ply.print();

    // 一時出力
    // obj_io.output_ply(pickup_img, default_dir_path + "pickup_img" + ".ply");
    // obj_io.output_ply(pickup_ply, default_dir_path + "pickup_ply" + ".ply");
    // obj_io.output_ply_with_line(pickup_img, default_dir_path + "pickup_img_line" + ".ply");

    // 方向ベクトルへconvert
    PointSet pickup_img_convert = calc.conversion_ply_to_img_point(pickup_img);
    pickup_img_convert.print();

    // plyファイルのほうは transform
    // Eigen::Vector3d transform_vec = {0, 0, 1.0};
    // pickup_ply.transform(transform_vec);
    // obj_io.output_ply(pickup_ply, default_dir_path + "pickup_ply_transform" + ".ply");

    // 理論値計算
    Eigen::Matrix3d Rironchi;
    Eigen::Vector3d rotate_axis = {1.0, 0, 1.0};
    double rotate_angle = 45.0;
    Rironchi = calc.calc_theory_value_Rotation_Matrix(rotate_axis, rotate_angle);

    // TODO もしかして ここの値代入していくやり方って数値計算的に良くないのか？？
    // 相関行列C
    // 相関行列Cは単位行列にする処理を含んでいるので、 convertしたのものを
    double weight = 1.0;
    Eigen::Matrix3d correlation_C = calc.calc_correlation_C(pickup_ply, pickup_img_convert, weight);
    // Eigen::Matrix3d correlation_C_origin = calc.calc_correlation_C(corresp_ply_point, corresp_imgply_point, weight);

    // 回転行列計算
    Eigen::Matrix3d rotation_matrix_R = calc.calc_rotation_matrix_from_correlation_c(correlation_C);
    // Eigen::Matrix3d rotation_matrix_R_origin = calc.calc_rotation_matrix_from_correlation_c(correlation_C_origin);

    // 回転軸・角度計算
    calc.calc_rotation_axis_from_matrix_R(rotation_matrix_R);
    // calc.calc_rotation_axis_from_matrix_R(rotation_matrix_R_origin);

    // 結果出力
    // corresp_ply_point.rotate(rotation_matrix_R);
    // corresp_ply_point.transform(transform_vec);
    // obj_io.output_ply(corresp_ply_point, default_dir_path + "ply_transform_rotate" + ".ply");

    // pickup_ply.rotate(rotation_matrix_R);
    // obj_io.output_ply(pickup_ply, default_dir_path + "pickup_ply_transform_rotate" + ".ply");

    // obj_io.output_ply(corresp_ply_point, default_dir_path + corresp_ply_point.get_name() + ".ply");
}

/**
 * @brief 読み込むファイル名, pathをprintする
 *
 */
void PointOperation::print()
{
    std::cout << std::endl
              << "default_dir_path: "
              << default_dir_path << std::endl;

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
