#include <iostream>
#include <fstream>
#include <string>

#include <vector>
#include <algorithm>
#include <typeinfo>
#include <float.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <eigen3/Core>
#include <eigen3/SVD>
#include <eigen3/LU>

// オプション処理
#include <unistd.h>

extern char *optarg;
extern int optind, opterr, optopt;

Eigen::Matrix3d calc_correlation_C(
    std::vector<Eigen::Vector3d> &x,
    std::vector<Eigen::Vector3d> &x_p,
    double weight)
{
    // vector3d は double型の三次元ベクトル
    std::vector<Eigen::Vector3d> m, m_p;

    // 単位ベクトルに変換する
    for (const Eigen::Vector3d &point_to_vec : x)
    {
        m.push_back(point_to_vec.normalized());
        // std::cout << point_to_vec.normalized() << std::endl;
    }
    for (const Eigen::Vector3d &point_to_vec : x_p)
    {
        m_p.push_back(point_to_vec.normalized());
        // std::cout << point_to_vec.normalized() << std::endl;
    }

    // 相関行列C
    Eigen::Matrix3d correlation_C = Eigen::Matrix3d::Identity();

    // 相関行列Cを求める
    //  a,bの数が一緒であることが前提
    for (auto iter = std::begin(m), iter_p = std::begin(m_p), last = std::end(m);
         iter != last; ++iter, ++iter_p)
    {
        // std::cout << correlation_C << std::endl;
        // ここイテレータのまま.transpose()とか使うことできないのかな。シンプルになるんだけど。
        Eigen::Vector3d tmp = *iter;
        Eigen::Vector3d tmp_p = *iter_p;

        // std::cout << "right" << std::endl
        //           << weight * tmp_p * (tmp.transpose()) << std::endl;

        correlation_C += weight * tmp_p * (tmp.transpose());
    }

    std::cout << std::endl
              << "correlation_C" << std::endl
              << correlation_C << std::endl;

    return correlation_C;
}

Eigen::Matrix3d calc_rotation_R(Eigen::Matrix3d correlation_C)
{
    // 特異値分解
    Eigen::JacobiSVD<Eigen::Matrix3d> SVD(correlation_C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d matrix_V, matrix_U, matrix_R;

    // https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html
    //  ここみると、SVD結果が A = USVで出力されている。
    //  計算は資料の方に合わせたいので、ここで反転させる。
    matrix_V = SVD.matrixU();
    matrix_U = SVD.matrixV();

    // det(V UT) 計算結果は 1か-1になるはず
    double det_VUt = (matrix_V * matrix_U.transpose()).determinant();

    // 対角行列
    Eigen::DiagonalMatrix<double, 3> matrix_Diag = {1.0, 1.0, det_VUt};

    // 回転行列Rの最大化の式
    matrix_R = matrix_V * matrix_Diag * matrix_U.transpose();

    // 回転行列R
    std::cout << std::endl
              << "Matrix_R" << std::endl;
    std::cout << std::setprecision(15) << matrix_R << std::endl
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

void SVD_test(Eigen::Matrix3d matrix_C)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> SVD(matrix_C, Eigen::ComputeFullU | Eigen::ComputeFullV);

    std::cout << "singular values" << std::endl;
    std::cout << SVD.singularValues() << std::endl;
    std::cout << "U matrix " << std::endl;
    std::cout << SVD.matrixU() << std::endl;
    std::cout << "V matrix " << std::endl;
    std::cout << SVD.matrixV() << std::endl;

    // 元の行列になるか。
    Eigen::Matrix3d M2;
    M2 = SVD.matrixU() * SVD.singularValues().asDiagonal() * SVD.matrixV().transpose();

    std::cout << "SVDでもとの行列になるか " << std::endl;
    std::cout << M2 << std::endl
              << std::endl;
    std::cout << "matrix_C" << std::endl
              << matrix_C << std::endl;
}

void load_pointdata(std::string file_name, std::string dir_path, int property_num, std::vector<Eigen::Vector3d> &point_data)
{
    std::fstream dat_file;
    // std::string dir_file = "./kyouiku-center/";
    std::string dir_file = dir_path;

    std::string file_path = dir_file + file_name;

    dat_file.open(file_path, std::ios::in);

    // 文字列分割についてはここ参考に
    //  https://marycore.jp/prog/cpp/std-string-split/
    std::string buffer;
    std::string separator = std::string(" ");
    auto separator_length = separator.length();

    while (std::getline(dat_file, buffer))
    {
        std::vector<std::string> buf_list;
        auto offset = std::string::size_type(0);

        while (1)
        {
            auto pos = buffer.find(separator, offset);
            if (pos == std::string::npos)
            {
                buf_list.push_back(buffer.substr(offset));
                break;
            }
            buf_list.push_back(buffer.substr(offset, pos - offset));
            offset = pos + separator_length;
        }

        // xyzだけを取り出したい
        // プロパティの数があっていて、# がついてないやつを読み込む。
        if (buf_list.size() == property_num)
        {
            if (buf_list.at(0) != "#")
            {
                std::vector<double> vec_data;
                for (auto e : buf_list)
                {
                    vec_data.push_back(std::stod(e));
                }
                Eigen::Vector3d tmp = {vec_data.at(0), vec_data.at(1), vec_data.at(2)};
                point_data.push_back(tmp);
            }
        }
    }

    std::cout << std::endl
              << "load ply_point" << std::endl;
    // for (auto e : point_data)
    // {
    //     std::cout << e(0) << " " << e(1) << " " << e(2) << std::endl;
    // }
}

Eigen::Vector3d equirectangular_to_sphere(double u, double v, double w, double h)
{
    // 正距円筒から球の座標への変換
    //  pointgetterからは、左上が原点の U,V座標で出力

    // 正規化
    u /= w;
    v /= h;

    // 緯度経度計算
    double phi = u * 2 * M_PI;
    double theta = v * M_PI;

    // double r = 1.0;

    // 方向ベクトル
    Eigen::Vector3d p = {abs(sin(theta)) * sin(phi), abs(sin(theta)) * cos(phi), cos(theta)};

    return p;
}

int load_img_pointdata(std::string file_name, std::string img_name, std::string dir_path, std::vector<Eigen::Vector3d> &point_data)
{
    std::fstream dat_file;
    // std::string dir_file = "./kyouiku-center/";
    std::string dir_file = dir_path;
    std::string img_file = "./img/";

    // 画像サイズが欲しいので 取ってくる。
    std::string img_file_path = img_file + img_name;
    cv::Mat img = cv::imread(img_file_path);
    if (img.empty())
    {
        std::cout << "couldn't read the image." << std::endl;
        return 1;
    }

    std::cout << std::endl
              << "img_size:height width" << std::endl;
    std::cout << img.rows << " " << img.cols << std::endl;

    std::string file_path = dir_file + file_name;

    dat_file.open(file_path, std::ios::in);

    // 文字列分割についてはここ参考に
    //  https://marycore.jp/prog/cpp/std-string-split/
    std::string buffer;
    std::string separator = std::string(" ");
    auto separator_length = separator.length();

    // image_point_dataは2つ
    int property_num = 2;

    std::cout << std::endl
              << "img_point_data" << std::endl;

    while (std::getline(dat_file, buffer))
    {
        std::vector<std::string> buf_list;
        auto offset = std::string::size_type(0);

        while (1)
        {
            auto pos = buffer.find(separator, offset);
            if (pos == std::string::npos)
            {
                buf_list.push_back(buffer.substr(offset));
                break;
            }
            buf_list.push_back(buffer.substr(offset, pos - offset));
            offset = pos + separator_length;
        }

        // xyzだけを取り出したい
        // 要素数が2つで# がついてないやつを読み込む。
        if (buf_list.size() == property_num)
        {
            if (buf_list.at(0) != "#")
            {
                std::vector<double> vec_data;
                for (auto e : buf_list)
                {
                    vec_data.push_back(std::stod(e));
                }

                std::cout << vec_data.at(0) << " " << vec_data.at(1) << std::endl;

                Eigen::Vector3d tmp = equirectangular_to_sphere(vec_data.at(0), vec_data.at(1), img.cols, img.rows);
                // Eigen::Vector3d tmp = {vec_data.at(0), vec_data.at(1), vec_data.at(2)};
                point_data.push_back(tmp);
            }
        }
    }

    std::cout << std::endl
              << "方向ベクトル変換後" << std::endl;
    for (auto e : point_data)
    {
        std::cout << e(0) << " " << e(1) << " " << e(2) << std::endl;
    }

    return 0;
}

Eigen::Matrix3d simlation_value(Eigen::Vector3d n, double degree)
{
    // 単位ベクトル
    n = n.normalized();

    double radian = degree * (M_PI / 180.0);

    Eigen::Matrix3d Matrix_R, Matrix_Kn, Matrix_Kn2, Matrix_I;

    Matrix_Kn << 0, -n(2), n(1),
        n(2), 0, -n(0),
        -n(1), n(0), 0;

    Matrix_I = Eigen::Matrix3d::Identity();
    Matrix_Kn2 = Matrix_Kn * Matrix_Kn;

    Matrix_R = Matrix_I + sin(radian) * Matrix_Kn + (1 - cos(radian)) * Matrix_Kn2;

    std::cout << "理論値" << std::endl;
    std::cout << std::setprecision(15) << Matrix_R << std::endl;

    return Matrix_R;
}

void rorate_pointdata(std::vector<Eigen::Vector3d> &point_data, std::vector<Eigen::Vector3d> &rotate_data, Eigen::Matrix3d matrix)
{
    for (Eigen::Vector3d &tmp : point_data)
    {
        tmp = matrix * tmp;
        rotate_data.push_back(tmp);
    }
}

void move_pointdata(std::vector<Eigen::Vector3d> &point_data, std::vector<Eigen::Vector3d> &move_data, double height)
{
    for (Eigen::Vector3d &tmp : point_data)
    {
        // tmp(0) = height + tmp(0);
        // tmp(1) = height + tmp(1);
        tmp(2) = height + tmp(2);

        move_data.push_back(tmp);
    }
}

void output_result(std::string file_path_1, std::string file_path_2, std::string out_path, Eigen::Matrix3d matrix)
{
    std::ofstream output_matrix(out_path, std::ios::app);

    output_matrix << std::endl;
    output_matrix << file_path_1 << " " << file_path_2 << std::endl;
    output_matrix << matrix;
}

void output_ply(std::vector<Eigen::Vector3d> &point_data, std::string out_path)
{
    // std::ios::app : 追記
    // std::ios::out : 書き込み
    std::ofstream output_ply(out_path, std::ios::out);

    output_ply << "ply" << std::endl
               << "format ascii 1.0" << std::endl
               << "element vertex " << point_data.size() << std::endl
               << "property float x" << std::endl
               << "property float y" << std::endl
               << "property float z" << std::endl
               << "end_header" << std::endl;

    for (const Eigen::Vector3d &tmp : point_data)
    {
        output_ply << tmp(0) << " " << tmp(1) << " " << tmp(2) << std::endl;
    }
}

void transform_coordinate(std::string file_path_1, std::string file_path_2, std::string file_path_3, std::string img_path, std::string out_path)
{

    std::vector<Eigen::Vector3d> x, x_prime, select_ply_point, plyfile_point;
    // std::string dir_path = "./kyoiku-2/";
    std::string dir_path = out_path;

    std::cout << std::fixed;

    // シミュレーション:load data
    //  load_pointdata(file_path_1, x);
    //  load_pointdata(file_path_2, x_prime);

    // 画像とpointdata
    load_img_pointdata(file_path_1, img_path, dir_path, x_prime);
    load_pointdata(file_path_2, dir_path, 3, select_ply_point);
    load_pointdata(file_path_3, dir_path, 4, plyfile_point);

    // 3Dデータとカメラとの距離(zのみを想定)
    double h = -1.0;
    move_pointdata(select_ply_point, x, h);

    // シミュレーション: 理論値
    // Eigen::Matrix3d Rironchi_matrix_R;
    // Eigen::Vector3d a = {0, 0, 1.0};
    // Rironchi_matrix_R = simlation_value(a, 30);

    // 重み
    double weight = 1;

    // 相関行列C
    Eigen::Matrix3d correlation_C;
    correlation_C = calc_correlation_C(x, x_prime, weight);

    // 回転行列計算
    Eigen::Matrix3d rotation_matrix_R;
    rotation_matrix_R = calc_rotation_R(correlation_C);

    output_ply(x_prime, out_path + "check-img.ply");
    output_ply(x, out_path + "check-ply.ply");

    std::vector<Eigen::Vector3d> rotated_plyfile_point, R_x;
    rorate_pointdata(plyfile_point, rotated_plyfile_point, rotation_matrix_R);

    output_ply(rotated_plyfile_point, out_path + "rotated_ply.ply");

    // SVD_test(correlation_C);
    // output_result(file_path_1, file_path_2, out_path, rotation_matrix_R);

    // rorate_pointdata(x, R_x, Rironchi_matrix_R);
    // rorate_pointdata(x_prime, i_x, Rironchi_matrix_R);

    // output_ply(R_x, out_path + "Rx.ply");
    // output_ply(i_x, out_path + "ix.ply");
}

int main(int argc, char *argv[])
{

    // コマンドオプション処理
    char opt;
    int i;

    std::string file_path_1;
    std::string file_path_2;
    std::string file_path_3;

    std::string out_path;
    std::string img_path;

    // コマンドライン引数のオプションがなくなるまで繰り返す
    //  getoputの第3引数にオプションの文字列を指定する。引数撮る場合は":"をつける
    //  a,cは引数をとらないが、 bは引数をとる。

    optind = 0;
    while ((opt = getopt(argc, argv, "i:")) != -1)
    {
        switch (opt)
        {
        case 'i':

            if (optarg[0] == '-')
            {
                printf("Option [%c] requires two arguments\n", opt);
                std::cout << "opt";
                return -1;
            }

            optind--;
            file_path_1 = argv[optind++];
            file_path_2 = argv[optind++];
            file_path_3 = argv[optind++];
            img_path = argv[optind++];
            out_path = argv[optind];

            // マジでオプション処理の動きがわからない
            //  for (i = 0; i < 1; i++)
            //  {

            //     if (argv[optind][0] == '-')
            //     {
            //         printf("error!");
            //         return -1;
            //     }
            //     file_path_2 = argv[optind++];
            // }

            printf("\n");
            break;
        default:
            printf("Unknown option '%c'\n", opt);
            break;
        }
        optarg = NULL;
    }
    /* オプション以外の引数を表示する。 */
    if (optind < argc)
    {
        while (optind < argc)
        {
            optind++;
            // printf("Not Option str '%s'\n", argv[optind++]);
        }
    }

    std::cout << "img_point :" << file_path_1 << std::endl;
    std::cout << "ply_point :" << file_path_2 << std::endl;
    std::cout << "plyfile :" << file_path_3 << std::endl
              << std::endl;
    std::cout << "imgpath :" << img_path << std::endl;
    std::cout << "outdir :" << out_path << std::endl;

    transform_coordinate(file_path_1, file_path_2, file_path_3, img_path, out_path);

    return 0;
}
