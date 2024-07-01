#include "calc_pointset.hpp"

void CalcPointSet::test_print()
{
    std::cout << "load calc pointset";
}

/**
 * @brief 回転行列の理論値を計算する。回転軸と角度を指定
 *
 * @param rotation_axis 回転軸
 * @param degree 回転角度 degree 
 * @return Eigen::Matrix3d 回転行列
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

    // std::cout << "理論値" << std::endl
    //           << Matrix_R << std::endl
    //           << std::endl;

    return Matrix_R;
}

/**
 * @brief img対応点とply対応点から 基本行列計算のためのvectorを作る
 *
 * @param img_point img対応点
 * @param ply_point ply対応点
 * @return std::vector<Eigen::Matrix<double, 9, 1>> xivector
 */
std::vector<Eigen::Matrix<double, 9, 1>> CalcPointSet::create_xi_vector(PointSet &img_point, PointSet &ply_point)
{
    std::cout << "create xi:" << std::endl;
    std::vector<Eigen::Vector3d>::const_iterator img_itr, ply_itr;
    img_point.print();
    std::vector<Eigen::Matrix<double, 9, 1>> vector_xi;

    for (
        img_itr = img_point.get_point_all().begin(), ply_itr = ply_point.get_point_all().begin();
        img_itr != img_point.get_point_all().end();
        ++img_itr, ++ply_itr)
    {
        // TODO: これいちいち呼び出さない方法はないですか？
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
 * @return Eigen::Matrix<double, 9, 9> martix_M
 */
Eigen::Matrix<double, 9, 9> CalcPointSet::calc_matrix_M(std::vector<Eigen::Matrix<double, 9, 1>> &vector_xi)
{

    std::cout << "----calc_matrix_M" << std::endl;
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
 * @param matrix_M 固有値分解する9×9の行列
 * @return Eigen::Matrix3d 基本行列
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
 * @param matrix_E 基本行列
 * @return Eigen::Vector3d 並進ベクトル
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
 * @brief 並進tの符号が異なってないかチェック。合ってなければ符号を変える。
 * TODO: 処理が合っているか分かってない。
 *
 * @param img_point 画像点の方向ベクトル
 * @param ply_point ply点群
 * @param vector_t 並進ベクトルt
 * @param matrix_e 基本行列E
 * @return Eigen::Vector3d 符号チェック後の並進ベクトル
 */
Eigen::Vector3d CalcPointSet::check_sign_translation_t(
    PointSet &img_point,
    PointSet &ply_point,
    Eigen::Vector3d &vector_t,
    Eigen::Matrix3d &matrix_e)
{

    std::cout << "-----check sign translation t" << std::endl;

    std::vector<Eigen::Vector3d>::const_iterator img_itr, ply_itr;
    double check_sign = 0;

    /**
     * @brief yが前にあるかどうか 決めて判定を行う
     *
     */
    auto is_vector_in_front = [](Eigen::Vector3d &img, Eigen::Vector3d &ply)
    {
        // y>0を前と決めてしまう。
        // TODO これどっちもy>0がいいのか、 imgだけでいいのか 分からんね。
        std::cout << "img_vec:" << std::endl
                  << img << std::endl;
        std::cout << "ply_vec:" << std::endl
                  << ply << std::endl;

        // signbitは minusだったら true返す
        if (std::signbit(img(1)) or std::signbit(ply(1)))
        {
            std::cout << "img_point y<0, exclusion " << std::endl;
            return false;
        }
        else
        {
            return true;
        }
    };

    // TODO 最初のアクセスだけおかしい値が出る。
    for (img_itr = img_point.get_point_all().begin() + 1, ply_itr = ply_point.get_point_all().begin() + 1;
         img_itr != img_point.get_point_all().end();
         ++img_itr, ++ply_itr)
    {
        Eigen::Vector3d img = *img_itr;
        Eigen::Vector3d ply = *ply_itr;

        bool print_check = is_vector_in_front(img, ply);
        std::cout << "print_check: " << print_check << std::endl;

        if (is_vector_in_front(img, ply))
        {
            auto Ex = matrix_e * ply;
            auto cross_M = img.cross(Ex);
            check_sign += vector_t.dot(cross_M);
            std::cout << "check_sign_NUM :" << vector_t.dot(cross_M) << std::endl;
        }
    }

    std::cout << "translation:: check_sign_sam: " << std::endl
              << check_sign << std::endl
              << std::endl;

    std::cout << "translation:: change_sign before" << std::endl
              << vector_t << std::endl;

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

    std::cout << "translation:: change_sign after" << std::endl
              << vector_t << std::endl;

    return vector_t;
}

/**
 * @brief 基本行列Eと並進ベクトルtから 回転行列R
 *
 * @param matrix_e 基本行列
 * @param vector_t 並進ベクトル
 * @return Eigen::Matrix3d 回転行列
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
    matrix_U = SVD.matrixV();
    matrix_V = SVD.matrixU();

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
    std::cout << "calc scale: " << std::endl;

    std::vector<Eigen::Vector3d>::const_iterator img_itr, ply_itr;

    double scale_s = 0.0;

    int scale_point_num = 0;

    std::cout << "vector_t: " << std::endl
              << vector_t << std::endl
              << "matrix_R:" << std::endl
              << matrix_R << std::endl;

    /**
     * @brief 並進と方向が一緒の場合を除くためにチェック
     *
     */
    auto is_cross_product_clone_to_zero = [](double calc_cross)
    {
        /**
         * @brief scaleの計算値が0に近いかどうかの基準値
         *
         */
        double delta = 0.001;
        if (calc_cross < delta)
        {
            std::cout << "------cross_product_clone_to_zero" << std::endl;
            return false;
        }
        else
        {
            return true;
        }
    };

    for (
        img_itr = img_point.get_point_all().begin() + 1, ply_itr = ply_point.get_point_all().begin() + 1;
        img_itr != img_point.get_point_all().end();
        ++img_itr, ++ply_itr)
    {
        Eigen::Vector3d img = *img_itr;
        Eigen::Vector3d ply = *ply_itr;

        double fraction_top = (matrix_R * ply).cross(img).dot(img.cross(vector_t));
        double fraction_bottom = img.cross(vector_t).squaredNorm();

        double scale_hat = fraction_top / fraction_bottom;

        if (is_cross_product_clone_to_zero(fraction_bottom))
        {
            scale_s += scale_hat;
            scale_point_num++;
        }
        // /** check 用
        // std::cout << "point:" << i++ << " " << scale_hat << std::endl;
        // std::cout << "img: " << std::endl
        //           << img << std::endl
        //           << "ply:" << std::endl
        //           << ply << std::endl;
        // std::cout << "fraction top: " << (matrix_R * ply).cross(img).dot(img.cross(vector_t)) << std::endl;
        // std::cout << "fraction bottom:" << img.cross(vector_t).squaredNorm() << std::endl;
        // std::cout << "scale_s_progress: " << scale_s << std::endl
        //           << std::endl;
        // */
    }

    scale_s /= double(scale_point_num);

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
 * @param weight よく分かってない
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
 * @param matrix_R 回転行列
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

    // ロドリゲスの式がうんたらかんたら
    double theta_rad = (matrix_R(0, 0) + matrix_R(1, 1) + matrix_R(2, 2) - (std::pow(nx, 2.0) + std::pow(ny, 2.0) + std::pow(nz, 2.0))) /
                       (3 - (std::pow(nx, 2.0) + std::pow(ny, 2.0) + std::pow(nz, 2.0)));

    theta_rad = std::acos(theta_rad);
    double theta_deg = theta_rad * 180.0 / M_PI;

    std::cout << "Rotation_degree: " << std::endl
              << theta_deg << std::endl
              << std::endl;
}

/**
 * @brief doubleの誤差を考慮した等価比較
 * https://marycore.jp/prog/c-lang/compare-floating-point-number/
 *
 * @param a
 * @param b
 * @return true : 等価
 * @return false : 等価じゃない
 */
bool check_float_equal(double a, double b)
{
    if (std::fabs(a - b) <= DBL_EPSILON * std::fmax(1, std::fmax(std::fabs(a), std::fabs(b))))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief 入力されたpoint_dataを単位球投影した方向ベクトルに変換する。
 *
 * @param point_data
 * @return PointSet
 */
PointSet CalcPointSet::conversion_ply_to_img_point(PointSet &point_data)
{
    PointSet img_point_data("conversion_unit_sphere");
    for (auto tmp : point_data.get_point_all())
    {

        if (check_float_equal(tmp(0), 0) && check_float_equal(tmp(1), 0) && check_float_equal(tmp(2), 0))
        {
            throw "0,0,0が含まれてます~~~~!!!!";
        }

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

/**
 * @brief Get the rand range object
 *
 * @param min_val 乱数の最小値
 * @param max_val 乱数の最大値
 * @return uint64_t 生成した乱数
 */
uint64_t get_rand_range(uint64_t min_val, uint64_t max_val)
{
    // 乱数生成器
    static std::mt19937_64 mt64(1);

    // [min_val, max_val] の一様分布整数 (int) の分布生成器
    std::uniform_int_distribution<uint64_t> get_rand_uni_int(min_val, max_val);

    // 乱数を生成
    return get_rand_uni_int(mt64);
}

/**
 * @brief ランダムな点番号を保存したpoint_dataを読み込み
 *
 * @param file_name point_filename
 * @param dir_path point_fileのディレクトリ
 * @param pickdata point_fileをベクターに格納したもの
 */
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

/**
 * @brief 与えられた数からランダムに選ぶ。 pick_numよりpicklimit(pointの総数)が小さければその範囲でエアr部
 *
 * @param out_path ランダムに選んだ数字を保存するpath
 * @param pick_num 何個pickupするか
 * @param pick_limit 0~pick_limitの間でランダムに選ぶ。
 */
void save_ramdom_pickup(std::string out_path, int pick_num, int pick_limit)
{

    // ramdomに選ぶ
    std::ofstream ramdom_ply(out_path, std::ios::out);

    if (pick_limit < pick_num)
    {
        pick_num = pick_limit;
    }

    for (int i = 0; i < pick_num; i++)
    {
        uint64_t pick_point = get_rand_range(0, pick_limit - 1);
        std::cout << pick_point << std::endl;
        ramdom_ply << pick_point << std::endl;
    }
}

/**
 * @brief シミュレーションデータ用にランダムに点をピックアップする。
 * ピックアップした後、点番号を保存する。
 * default_dirに保存したpoint_fileがあれば、そのデータを使用、なければ新しくピックアップする。
 *  修正: dirを指定する場合と しない場合で調整
 *
 * @param point_data
 * @param point_data2
 * @param pickup_data
 * @param pickup_data2
 * @param pickup_file_path
 */
void CalcPointSet::pickup_corresp_point(
    PointSet &point_data, PointSet &point_data2,
    PointSet &pickup_data, PointSet &pickup_data2,
    std::string pickup_file_path)
{
    std::cout << "-------pickup point :" << std::endl;
    
    // 指定したディレクトリに存在するか？
    bool is_file_exists = std::filesystem::exists(pickup_file_path + "pickup_num.dat");

    // ファイルがない場合 新しくpoint_fileを作る。
    if (is_file_exists == false)
    {
        std::cout << "no pickup file -> make_point_data" << std::endl;

        std::string out_path = "pickup_num.dat";
        int pickup_num = 30;
        int pickup_limit = int(point_data.get_point_num());

        save_ramdom_pickup(out_path, pickup_num, pickup_limit);
    }

    // ファイルを読み込む
    std::vector<int> ramdom_pickup;
    load_ramdom_data("pickup_num.dat", pickup_file_path, ramdom_pickup);

    // 読み込んだ点を格納
    for (const auto pick_num : ramdom_pickup)
    {
        std::cout << point_data.get_point(pick_num).transpose() << " " << std::endl;
        pickup_data.add_point(point_data.get_point(pick_num));
        pickup_data2.add_point(point_data2.get_point(pick_num));
    }
}

/**
 * @brief 2つの点群を
 *
 */
void CalcPointSet::make_striped_pattern(std::vector<int> &stitch_edge, PointSet &stitch_edge_point, PointSet &watermelon, int divide_num)
{
    double r = 1.0;

    //
    bool flag = true;
    // suicaの縞模様のphi_間隔
    int phi_step = 360 / divide_num;
    // int theta_step = 180 / divide_num;
    int theta_step = 0;
    int i = 0;

    for (int phi_angle = 0; phi_angle < 2 * 180; phi_angle++)
    {
        if (i < phi_step)
        {
            int j = 0;
            for (int theta_angle = 0; theta_angle < 180; theta_angle++)
            {
                if ((i == 0 || (i == phi_step - 1)) && j == theta_step && flag)
                {
                    stitch_edge_point.add_point_polar(Eigen::Vector3d{r, theta_angle * M_PI / 180, phi_angle * M_PI / 180});
                    stitch_edge.at(phi_angle * 180 + theta_angle) = 1;
                }
                else
                {
                    stitch_edge.at(phi_angle * 180 + theta_angle) = 0;
                }
                if (flag)
                {
                    watermelon.add_point_polar(Eigen::Vector3d{r, theta_angle * M_PI / 180, phi_angle * M_PI / 180});
                }
                // test_point.add_point(Eigen::Vector3d(sin(theta_angle * M_PI / 180) * cos(phi_angle * M_PI / 180), sin(theta_angle * M_PI / 180) * sin(phi_angle * M_PI / 180), cos(theta_angle * M_PI / 180)));
                if (j < theta_step)
                {
                    j++;
                }
                else
                {
                    j = 0;
                }
            }
            i++;
        }
        else
        {
            i = 0;
            flag = !flag;
        }
    }
}
