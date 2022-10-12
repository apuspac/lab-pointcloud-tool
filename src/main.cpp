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

    //相関行列C
    Eigen::Matrix3d correlation_C = Eigen::Matrix3d::Identity();

    //相関行列Cを求める
    // a,bの数が一緒であることが前提
    for (auto iter = std::begin(m), iter_p = std::begin(m_p), last = std::end(m);
         iter != last; ++iter, ++iter_p)
    {
        // std::cout << correlation_C << std::endl;
        // ここイテレータのまま.transpose()とか使うことできないのかな。シンプルになるんだけど。
        Eigen::Vector3d tmp = *iter;
        Eigen::Vector3d tmp_p = *iter_p;

        correlation_C += weight * tmp_p * (tmp.transpose());
    }

    return correlation_C;
}

Eigen::Matrix3d calc_rotation_R(Eigen::Matrix3d correlation_C)
{
    // 特異値分解
    Eigen::JacobiSVD<Eigen::Matrix3d> SVD(correlation_C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d matrix_V, matrix_U, matrix_R;

    matrix_U = SVD.matrixU();
    matrix_V = SVD.matrixV();

    // det(V UT) 計算結果は 1か-1になるはず
    double det_VU = (matrix_V * matrix_U.transpose()).determinant();

    //対角行列
    Eigen::DiagonalMatrix<double, 3> matrix_Diag = {1.0, 1.0, det_VU};

    // 回転行列Rの最大化の式
    matrix_R = matrix_V * matrix_Diag * matrix_U.transpose();

    //回転行列R
    std::cout << "Matrix_R" << std::endl;
    std::cout << matrix_R << std::endl
              << std::endl;

    // TODO check方法をもうちょっと工夫する 許容範囲の誤差以外になったらはじくとか。
    std::cout << "check R_top*R = R*R_top = I" << std::endl;
    std::cout << matrix_R.transpose() * matrix_R << std::endl
              << std::endl;
    std::cout << matrix_R * matrix_R.transpose() << std::endl
              << std::endl;

    double r_det = matrix_R.determinant();

    std::cout << "detR = +1 check" << std::endl;
    std::cout << r_det << std::endl
              << std::endl;

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
}

void load_pointdata(std::string file_name, std::vector<Eigen::Vector3d> &point_data)
{
    std::fstream dat_file;
    std::string point_file = "./point_data/";

    std::string file_path = point_file + file_name;

    dat_file.open(file_path, std::ios::in);

    //文字列分割についてはここ参考に
    // https://marycore.jp/prog/cpp/std-string-split/
    std::string buffer;
    std::string separator = std::string(" ");
    auto separator_length = separator.length();

    int property_num = 3;

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
        //要素数が3つで# がついてないやつを読み込む。
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
}

Eigen::Vector3d equirectangular_to_sphere(double u, double v, double w, double h)
{
    //正距円筒から球の座標への変換
    double phi = u * (2 * M_PI) / w;
    double theta = v * (M_PI / h);
    double r = 1.0;

    Eigen::Vector3d p = {r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta)};

    return p;
}

void load_img_pointdata(std::string file_name, std::string img_name, std::vector<Eigen::Vector3d> &point_data)
{
    std::fstream dat_file;
    std::string point_file = "./point_data/";
    std::string img_file = "./img/";

    //画像サイズが欲しいので 取ってくる。
    std::strint img_file_path = img_file + img_name;
    cv::Mat img = cv::imread(img_file_path);
    if (img.empty())
    {
        cout << "couldn't read the image." << endl;
        return 1;
    }

    std::string file_path = point_file + file_name;

    dat_file.open(file_path, std::ios::in);

    //文字列分割についてはここ参考に
    // https://marycore.jp/prog/cpp/std-string-split/
    std::string buffer;
    std::string separator = std::string(" ");
    auto separator_length = separator.length();

    int property_num = 3;

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
        //要素数が3つで# がついてないやつを読み込む。
        if (buf_list.size() == property_num)
        {
            if (buf_list.at(0) != "#")
            {
                std::vector<double> vec_data;
                for (auto e : buf_list)
                {
                    vec_data.push_back(std::stod(e));
                }

                Eigen::Vector3d tmp = equirectangular_to_sphere(vec_data.at(0), vec_data.at(1), img.rows, img.cols);
                // Eigen::Vector3d tmp = {vec_data.at(0), vec_data.at(1), vec_data.at(2)};
                point_data.push_back(tmp);
            }
        }
    }
}

void output_result(std::string file_path_1, std::string file_path_2, std::string out_path, Eigen::Matrix3d matrix)
{
    std::ofstream output_matrix(out_path, std::ios::app);

    output_matrix << file_path_1 << " " << file_path_2 << std::endl;
    output_matrix << matrix;
}

// vertex# 1198
// position [29.282000 109.281998 100.000000]
// normal [0.000000 0.000000 0.000000]
// color (0.000000 0.000000 0.000000 0.000000)
// ------
// ------
// vertex# 1330
// position [36.602501 136.602997 100.000000]
// normal [0.000000 0.000000 0.000000]
// color (0.000000 0.000000 0.000000 0.000000)
// ------
// ------
// vertex# 1162
// position [59.282001 57.320499 40.000000]
// normal [0.000000 0.000000 0.000000]
// color (0.000000 0.000000 0.000000 0.000000)
// ------
// ------
// vertex# 1087
// position [1.961520 116.602997 80.000000]
// normal [0.000000 0.000000 0.000000]
// color (0.000000 0.000000 0.000000 0.000000)
// ------

void transform_coordinate(std::string file_path_1, std::string file_path_2, std::string out_path)
{

    std::vector<Eigen::Vector3d> x, x_p;
    // std::string file_path_1 = "./point_data/res-data.dat";
    // std::string file_path_2 = "./point_data/res-data-rotate.dat";

    load_pointdata(file_path_1, x);
    load_pointdata(file_path_2, x_p);

    // 重み
    double weight = 1.0;
    //相関行列C
    Eigen::Matrix3d correlation_C;
    correlation_C = calc_correlation_C(x, x_p, weight);

    std::cout << "correlation_C : " << std::endl;
    std::cout << correlation_C << std::endl
              << std::endl;

    // SVD_test(correlation_C);
    Eigen::Matrix3d matrix_R;
    matrix_R = calc_rotation_R(correlation_C);

    output_result(file_path_1, file_path_2, out_path, matrix_R);
}

int main(int argc, char *argv[])
{

    //コマンドオプション処理
    char opt;
    int i;

    std::string file_path_1;
    std::string file_path_2;
    std::string out_path;

    //コマンドライン引数のオプションがなくなるまで繰り返す
    // getoputの第3引数にオプションの文字列を指定する。引数撮る場合は":"をつける
    // a,cは引数をとらないが、 bは引数をとる。

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
            out_path = argv[optind];

            //マジでオプション処理の動きがわからない
            // for (i = 0; i < 1; i++)
            // {

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

    std::cout << "filepath_1 :" << file_path_1 << std::endl;
    std::cout << "filepath_2 :" << file_path_2 << std::endl
              << std::endl;
    // std::cout << "outpath :" << out_path << std::endl;

    transform_coordinate(file_path_1, file_path_2, out_path);

    return 0;
}
