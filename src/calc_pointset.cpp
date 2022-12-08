#include "calc_pointset.hpp"

void CalcPointSet::test_print()
{
    std::cout << "load calc pointset";
}

/**
 * @brief 回転行列の理論値を計算する。回転軸と角度を指定
 *
 * @param rotation_axis
 * @param degree
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d CalcPointSet::calc_theory_value_Rotation_Matrix(Eigen::Vector3d rotation_axis, double degree)
{

    Eigen::Vector3d n = rotation_axis.normalized();

    double radian = degree * (M_PI / 180.0);

    Eigen::Matrix3d Matrix_R, Matrix_Kn, Matrix_Kn2, Matrix_I;

    Matrix_Kn << 0, -n(2), n(1),
        n(2), 0, -n(0),
        -n(1), n(0), 0;
    Matrix_I = Eigen::Matrix3d::Identity();
    Matrix_Kn2 = Matrix_Kn * Matrix_Kn;

    // ロドリゲスの回転公式
    Matrix_R = Matrix_I + sin(radian) * Matrix_Kn + (1 - cos(radian)) * Matrix_Kn2;

    std::cout << "理論値" << std::endl
              << Matrix_R << std::endl
              << std::endl;

    return Matrix_R;
}

/**
 * @brief img対応点とply対応点から vectorを作る
 *
 * @param img_point
 * @param ply_point
 * @return std::vector<Eigen::Matrix<double, 9, 1>>
 */
std::vector<Eigen::Matrix<double, 9, 1>> CalcPointSet::create_xi_vector(PointSet &img_point, PointSet &ply_point)
{
    std::vector<Eigen::Vector3d>::const_iterator img_itr, ply_itr;
    img_point.print();
    std::vector<Eigen::Matrix<double, 9, 1>> vector_xi;

    // TODO begin()+1としないと 1個目の取得が変な値になってしまうのはなぜ。
    for (
        img_itr = img_point.get_point_all().begin() + 1, ply_itr = ply_point.get_point_all().begin() + 1;
        img_itr != img_point.get_point_all().end();
        ++img_itr, ++ply_itr)
    {
        // TODO: いちいち呼び出さない方法はないですか？
        Eigen::Vector3d img = *img_itr;
        Eigen::Vector3d ply = *ply_itr;

        double x = img(0);
        double y = img(1);
        double z = img(2);
        double x_p = ply(0);
        double y_p = ply(1);
        double z_p = ply(2);

        Eigen::Matrix<double, 9, 1> xi = {
            x * x_p, x * y_p, x * z_p,
            y * x_p, y * y_p, y * z_p,
            z * x_p, z * y_p, z * z_p};

        vector_xi.push_back(xi);
    }

    std::cout << "vector_xi.transpose() :" << std::endl;
    for (const auto &tmp : vector_xi)
    {
        std::cout << tmp.transpose() << std::endl;
    }
    std::cout << std::endl;

    return vector_xi;
}

/**
 * @brief 行列Mを計算する。
 *
 * @param vector_xi
 * @return Eigen::Matrix<double, 9, 9>
 */
Eigen::Matrix<double, 9, 9> CalcPointSet::calc_matrix_M(std::vector<Eigen::Matrix<double, 9, 1>> &vector_xi)
{
    Eigen::Matrix<double, 9, 9> Matrix_M = Eigen::Matrix<double, 9, 9>::Identity();

    for (const auto &xi : vector_xi)
    {
        Matrix_M += xi * xi.transpose();
    }

    std::cout << "Matrix_M:" << std::endl
              << Matrix_M << std::endl
              << std::endl;
    return Matrix_M;
}

/**
 * @brief 行列Mを固有値分解し、 最小固有値に対する固有ベクトルを並べて、基本行列とする。
 *
 * @param matrix_M
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d CalcPointSet::calc_essential_matrix(Eigen::Matrix<double, 9, 9> &matrix_M)
{
    // 固有値を計算
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> ES(matrix_M);

    if (ES.info() != Eigen::Success)
    {
        abort();
    }

    std::cout << "Essential_Matrix:: Matrix M Eigenvalue :" << std::endl
              << ES.eigenvalues() << std::endl
              << std::endl;

    std::cout << "Essential_Matrix:: Matrix M EigenVector: " << std::endl
              << ES.eigenvectors() << std::endl
              << std::endl;

    // 最小固有値
    double min_eigen = ES.eigenvalues()(0);

    std::cout << "Essential_Matrix:: min_eigen" << std::endl
              << min_eigen << std::endl
              << std::endl;

    // 最小固有値に対応するベクトル
    std::cout << "Essential_Matrix:: min_eigenvector" << std::endl
              << ES.eigenvectors().col(0) << std::endl
              << std::endl;

    Eigen::Matrix<double, 9, 1> min_eigen_vector = ES.eigenvectors().col(0);

    Eigen::Matrix3d Matrix_E;

    Matrix_E << min_eigen_vector(0), min_eigen_vector(1), min_eigen_vector(2),
        min_eigen_vector(3), min_eigen_vector(4), min_eigen_vector(5),
        min_eigen_vector(6), min_eigen_vector(7), min_eigen_vector(8);

    std::cout << "Essential Matrix:" << std::endl
              << Matrix_E << std::endl
              << std::endl;

    return Matrix_E;
}

/**
 * @brief スケールを除いた並進ベクトルtを基本行列Eから求める
 *
 * @param matrix_E
 * @return Eigen::Vector3d
 */
Eigen::Vector3d CalcPointSet::calc_translation_t(Eigen::Matrix3d &matrix_E)
{
    Eigen::Matrix3d matrix_EEt = matrix_E * matrix_E.transpose();

    // 固有値を計算
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ES(matrix_EEt);

    if (ES.info() != Eigen::Success)
    {
        abort();
    }
    // 固有値・固有ベクトル
    std::cout << std::endl
              << "Translation:: E * E.t Eigenvalue :" << std::endl
              << ES.eigenvalues() << std::endl
              << std::endl;
    std::cout << "Translation:: E * E.t Eigenvec :" << std::endl
              << ES.eigenvectors() << std::endl
              << std::endl;

    // 最小固有値・固有ベクトル
    double min_eigen = ES.eigenvalues()(0);
    Eigen::Vector3d vector_t_hat = ES.eigenvectors().col(0);
    std::cout << "Translation:: E * E.t min_eigen" << std::endl
              << min_eigen << std::endl
              << std::endl;
    std::cout << "Translation:: E * E.t min_eigenvector" << std::endl
              << vector_t_hat << std::endl
              << std::endl;

    // Eigen::Vector3d translation_diff_scale = {vector_t_hat(0), vector_t_hat(1), vector_t_hat(2)};

    std::cout << "Translation:: diff scale" << std::endl
              << vector_t_hat << std::endl
              << std::endl;

    return vector_t_hat;
}

/**
 * @brief 並進tの符号が異なってないかチェック
 *
 * @param img_point
 * @param ply_point
 * @param vector_t
 * @param matrix_e
 * @return Eigen::Vector3d
 */
Eigen::Vector3d CalcPointSet::check_sign_translation_t(
    PointSet &img_point,
    PointSet &ply_point,
    Eigen::Vector3d &vector_t,
    Eigen::Matrix3d &matrix_e)
{
    std::vector<Eigen::Vector3d>::const_iterator img_itr, ply_itr;
    double check_sign = 0;
    for (img_itr = img_point.get_point_all().begin() + 1, ply_itr = ply_point.get_point_all().begin() + 1;
         img_itr != img_point.get_point_all().end();
         ++img_itr, ++ply_itr)
    {
        Eigen::Vector3d img = *img_itr;
        Eigen::Vector3d ply = *ply_itr;

        auto Ex = matrix_e * ply;

        auto cross_M = img.cross(Ex);

        check_sign += vector_t.dot(cross_M);
    }

    std::cout << "translation:: check_sign: " << std::endl
              << check_sign << std::endl
              << std::endl;

    auto is_large_than_zero = [](double check_num)
    {
        if (check_num > 0)
            return true;
        else
            return false;
    };

    if (!(is_large_than_zero(check_sign)))
    {
        vector_t = -vector_t;
    }

    std::cout << "translation:: change_sign" << std::endl
              << vector_t << std::endl;

    return vector_t;
}

/**
 * @brief 基本行列Eと並進ベクトルtから 回転行列R
 *
 * @param matrix_e
 * @param vector_t
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d CalcPointSet::calc_rotation_matrix_from_essential_matrix(Eigen::Matrix3d &matrix_e, Eigen::Vector3d &vector_t)
{
    Eigen::Matrix3d vector_t_cross;

    vector_t_cross << 0, -vector_t(2), vector_t(1),
        vector_t(2), 0, -vector_t(0),
        -vector_t(1), vector_t(0), 0;

    std::cout << std::endl
              << "translation_t_cross: " << std::endl
              << vector_t_cross << std::endl
              << std::endl;

    Eigen::Matrix3d matrix_K = -vector_t_cross * matrix_e;

    // 特異値分解
    Eigen::JacobiSVD<Eigen::Matrix3d> SVD(matrix_K, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d matrix_V, matrix_U, matrix_R;

    // https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html
    //  ここみると、SVD結果が A = USVで出力されている。
    //  計算は資料の方に合わせたいので、ここで反転させる。
    matrix_V = SVD.matrixV();
    matrix_U = SVD.matrixU();

    // det(V UT) 計算結果は 1か-1になるはず
    double det_VUt = (matrix_V * matrix_U.transpose()).determinant();

    // 対角行列
    Eigen::DiagonalMatrix<double, 3> matrix_Diag = {1.0, 1.0, det_VUt};

    // 回転行列Rの最大化の式
    matrix_R = matrix_V * matrix_Diag * matrix_U.transpose();

    // 回転行列R
    std::cout << "Matrix_R calc from matrix_E: " << std::endl;
    std::cout << matrix_R << std::endl
              << std::endl;

    return matrix_R;
}

/**
 * @brief 並進tのスケールの推定
 *
 * @param img_point
 * @param ply_point
 * @param matrix_R
 * @param vector_t
 * @return double
 */
double CalcPointSet::calc_scale_of_translation_t(
    PointSet &img_point, PointSet &ply_point,
    Eigen::Matrix3d &matrix_R, Eigen::Vector3d &vector_t)
{

    std::vector<Eigen::Vector3d>::const_iterator img_itr, ply_itr;

    double scale_s = 0.0;
    for (
        img_itr = img_point.get_point_all().begin() + 1, ply_itr = ply_point.get_point_all().begin() + 1;
        img_itr != img_point.get_point_all().end();
        ++img_itr, ++ply_itr)
    {
        Eigen::Vector3d img = *img_itr;
        Eigen::Vector3d ply = *ply_itr;

        // std::cout << "m" << img << std::endl
        //           << "x" << ply << std::endl
        //           << "t" << vector_t << std::endl
        //           << "R" << matrix_R << std::endl;

        // Eigen::Vector3d Rx_cross_m = (matrix_R * ply).cross(img);
        // Eigen::Vector3d m_cross_t = img.cross(vector_t);

        // double frac_up = Rx_cross_m.dot(m_cross_t);
        // double frac_down = vector_t.cross(img).squaredNorm();

        // scale_s += frac_up / frac_down;

        scale_s += (matrix_R * ply).cross(img).dot(img.cross(vector_t)) / img.cross(vector_t).squaredNorm();

        // std::cout << "Rx_cross_m" << Rx_cross_m << std::endl
        //           << "m_cross_t" << m_cross_t << std::endl
        //           << "frac_up" << frac_up << std::endl
        //           << "frac_down" << frac_down << std::endl
        //           << "scale_s 推定" << (matrix_R * ply).cross(img).dot(img.cross(vector_t)) / img.cross(vector_t).squaredNorm() << std::endl
        //           << "scale_s sum" << scale_s << std::endl
        //           << "img_point_num" << double(img_point.get_point_num()) << std::endl
        //           << std::endl;
    }

    // iterがなぜか+1しないと nanが出てしまうので-1している
    scale_s /= double(img_point.get_point_num() - 1);

    // 測定誤差を考慮しないとき
    // Eigen::Vector3d rx_cross_m = (matrix_R * ply_point.get_point(1)).cross(img_point.get_point(1));
    // Eigen::Vector3d m_cross_t = img_point.get_point(1).cross(vector_t);

    // double scale_s = rx_cross_m.dot(m_cross_t) / img_point.get_point(1).cross(vector_t).squaredNorm();

    std::cout << "scale_s" << std::endl
              << scale_s << std::endl
              << std::endl;

    return scale_s;
}

/**
 * @brief 回転前後の点群の組から 回転行列を生成するための相関行列の計算
 *  x_p = Rx
 * @param x 座標を合わせる(回転行列をかけて回転させる)点群。 plyfile
 * @param x_p 座標を合わせられる点群。 imgfile
 * @param weight
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d CalcPointSet::calc_correlation_C(
    PointSet &x,
    PointSet &x_p,
    double weight)
{

    std::vector<Eigen::Vector3d> m, m_p;

    // 単位行列の変換
    for (const Eigen::Vector3d &point_to_vec : x.get_point_all())
    {
        // m.push_back(point_to_vec.normalized());
        m.push_back(point_to_vec);
    }
    for (const Eigen::Vector3d &point_to_vec : x_p.get_point_all())
    {
        // m_p.push_back(point_to_vec.normalized());
        m_p.push_back(point_to_vec);
    }

    // 相関行列C
    Eigen::Matrix3d correlation_C = Eigen::Matrix3d::Identity();

    // 相関行列Cを求める
    for (auto iter = std::begin(m) + 1, iter_p = std::begin(m_p) + 1, last = std::end(m);
         iter != last; ++iter, ++iter_p)
    {
        Eigen::Vector3d tmp = *iter;
        Eigen::Vector3d tmp_p = *iter_p;

        correlation_C += weight * tmp_p * (tmp.transpose());
    }

    std::cout << std::endl
              << "correlation_C" << std::endl
              << correlation_C << std::endl
              << std::endl;

    return correlation_C;
}

/**
 * @brief 相関行列から回転角を計算する
 *
 * @param correlation_C
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d CalcPointSet::calc_rotation_matrix_from_correlation_c(Eigen::Matrix3d &correlation_C)
{
    // 特異値分解
    Eigen::JacobiSVD<Eigen::Matrix3d> SVD(correlation_C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d matrix_V, matrix_U, matrix_R;

    // https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html
    //  ここみると、SVD結果が A = USVで出力
    matrix_V = SVD.matrixV();
    matrix_U = SVD.matrixU();

    // det(V UT) 計算結果は 1か-1になるはず
    double det_VUt = (matrix_V * matrix_U.transpose()).determinant();

    // 対角行列
    Eigen::DiagonalMatrix<double, 3> matrix_Diag = {1.0, 1.0, det_VUt};

    // 回転行列Rの最大化の式
    matrix_R = matrix_V * matrix_Diag * matrix_U.transpose();

    // 回転行列R
    std::cout << std::endl
              << "Matrix_R" << std::endl
              << matrix_R << std::endl
              << std::endl;

    // // TODO check方法をもうちょっと工夫する 許容範囲の誤差以外になったらはじくとか。
    // std::cout << "check R_top*R = R*R_top = I" << std::endl;
    // std::cout << matrix_R.transpose() * matrix_R << std::endl
    //           << std::endl;
    // std::cout << matrix_R * matrix_R.transpose() << std::endl
    //           << std::endl;

    // double r_det = matrix_R.determinant();

    // std::cout << "detR = +1 check" << std::endl;
    // std::cout << r_det << std::endl
    //           << std::endl;

    return matrix_R;
}

/**
 * @brief 回転行列から回転軸と回転角を計算する
 *
 * @param matrix_R
 */
void CalcPointSet::calc_rotation_axis_from_matrix_R(Eigen::Matrix3d &matrix_R)
{

    // 固有値を計算
    Eigen::EigenSolver<Eigen::Matrix3d> ES(matrix_R);

    if (ES.info() != Eigen::Success)
    {
        abort();
    }

    // (実部, 虚部)
    // ここで1に対応するのが固有ベクトルが回転軸
    // std::cout << "Rotation matrix Eigenvalue :" << std::endl
    //           << ES.eigenvalues() << std::endl;

    // 固有ベクトル
    // std::cout << "Eigen vec" << std::endl
    //           << ES.eigenvectors() << std::endl;

    // 1に近いやつを探す
    double min_eigen = std::abs(1.0 - ES.eigenvalues()(0).real());

    long int index = 0;
    for (const auto &tmp : ES.eigenvalues())
    {
        if (std::abs(1.0 - tmp.real()) < min_eigen)
        {
            min_eigen = std::abs(1.0 - tmp.real());
            index = &tmp - &ES.eigenvalues()(0);
        }
    }

    Eigen::Vector3d vector_t = ES.eigenvectors().col(index).real();

    std::cout << "Rotation_axis" << std::endl
              << vector_t << std::endl
              << std::endl;

    // 回転角の計算
    double nx = vector_t(0);
    double ny = vector_t(1);
    double nz = vector_t(2);

    double theta_rad = (matrix_R(0, 0) + matrix_R(1, 1) + matrix_R(2, 2) - (std::pow(nx, 2.0) + std::pow(ny, 2.0) + std::pow(nz, 2.0))) /
                       (3 - (std::pow(nx, 2.0) + std::pow(ny, 2.0) + std::pow(nz, 2.0)));

    theta_rad = std::acos(theta_rad);
    double theta_deg = theta_rad * 180.0 / M_PI;

    std::cout << "Rotation_degree: " << std::endl
              << theta_deg << std::endl
              << std::endl;
}

PointSet CalcPointSet::conversion_ply_to_img_point(PointSet &point_data)
{
    PointSet img_point_data("ply_to_img");
    for (auto tmp : point_data.get_point_all())
    {
        double theta = std::acos(
            tmp(2) /
            std::sqrt(std::pow(tmp(0), 2.0) + std::pow(tmp(1), 2.0) + std::pow(tmp(2), 2.0)));

        double phi = std::atan2(tmp(1), tmp(0));

        double r = 1.0;

        // 方向ベクトル
        Eigen::Vector3d p = {r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta)};

        img_point_data.add_point(p);
    }

    return img_point_data;
}

uint64_t get_rand_range(uint64_t min_val, uint64_t max_val)
{
    // 乱数生成器
    static std::mt19937_64 mt64(0);

    // [min_val, max_val] の一様分布整数 (int) の分布生成器
    std::uniform_int_distribution<uint64_t> get_rand_uni_int(min_val, max_val);

    // 乱数を生成
    return get_rand_uni_int(mt64);
}

void load_ramdom_data(std::string file_name, std::string dir_path, std::vector<int> &pickdata)
{
    std::fstream data_file;
    std::string file_path = dir_path + file_name;

    data_file.open(file_path, std::ios::in);

    std::string one_line_buffer;

    // getlineで1行ずつ処理する
    while (std::getline(data_file, one_line_buffer))
    {
        pickdata.push_back(std::stoi(one_line_buffer));
    }

    for (const auto tmp : pickdata)
    {
        std::cout << tmp << " ";
    }
}

// TODO ランダム生成して保存する場合と 生成したやつを読み込むものに分けたい。
void CalcPointSet::pickup_corresp_point(PointSet &point_data, PointSet &point_data2, PointSet &pickup_data, PointSet &pickup_data2)
{
    std::cout << "pickup point :" << std::endl;

    std::vector<int> ramdom_pickup;
    load_ramdom_data("pickup_num.dat", "../../ply_data/", ramdom_pickup);

    // 読み込む場合
    for (const auto pick_num : ramdom_pickup)
    {
        pickup_data.add_point(point_data.get_point(pick_num));
        pickup_data2.add_point(point_data2.get_point(pick_num));
    }

    // ramdomに選ぶ場合
    // for (int i = 0; i < 30; i++)
    // {

    //     uint64_t pick_num = get_rand_range(0, point_data.get_point_num());
    //     std::cout << pick_num << std::endl;

    //     pickup_data.add_point(point_data.get_point(pick_num));
    //     pickup_data2.add_point(point_data2.get_point(pick_num));

    //     // pickup_point.add_point(point_data.get_point()));
    // }
}
/**

**/