#include "object_io.hpp"

/**
 * @brief オプション処理
 *
 * @param argc int main()の引数のargc
 * @param argv char* main()の引数のargv
 * @return int 正常終了したか // TODO 正常終了したかのチェックを作っていない
 */
void ObjectIO::option_process(int argc, char **argv, PointOperation &operation)
{

    // m: imgの対応点 .dat
    // x: plyの対応点 .dat
    // p: plyファイル .ply
    // i: imgファイル .jpg,png
    // d: デフォルトdirpath
    // h: help
    // 複数入力をさせたいけど ちょっと面倒っぽいので、 2回指定するときは、2回とも入力する。
    const char *opt_string = "m:x:p:d:i:h";

    // オプション処理のlong用
    // {*name,  has_arg,    *flag,  val},
    // has_argは 引数が必要なら has_arg, 必要なかったらno_arg どっちでもよいときはoptional_arg
    // flag 0 のときは valが返される
    static struct option long_options[] =
        {
            {"img_cp", required_argument, 0, 'm'},
            {"ply_cp", required_argument, 0, 'x'},
            {"ply", required_argument, 0, 'p'},
            {"img", required_argument, 0, 'i'},
            {"dir", required_argument, 0, 'd'},
            {"help", no_argument, 0, 'h'},

        };

    int opt;
    opterr = 0;
    int option_index = 0;

    while ((opt = getopt_long(argc, argv, opt_string, long_options, &option_index)) != -1)
    {
        switch (opt)
        {
        case 'm':
            operation.add_corresp_img_file_name(std::string(optarg));
            break;

        case 'x':
            operation.add_corresp_ply_file_name(std::string(optarg));
            break;
            ;
        case 'p':
            operation.add_plyfile_name(std::string(optarg));
            break;

        case 'i':
            operation.add_img_file_path(std::string(optarg));
            break;

        case 'd':
            operation.set_default_dir_path(std::string(optarg));
            break;
        case 'h':
            std::cout << "help" << std::endl
                      << "--img_cp,  -m : corresp img point data filename" << std::endl
                      << "--ply_cp,   -x : corresp ply point data filename" << std::endl
                      << "--ply,      -p : ply file name " << std::endl
                      << "--img,      -m : img file path" << std::endl
                      << "--dir,      -d : dir file path" << std::endl;
            break;
        default:
            break;
        }
    }
}

/**
 * @brief plyファイルと その対応点を作る。
 *
 * @param file_name 入力plyファイル名
 * @param dir_path 入力plyファイルのあるディレクトリ
 * @param property_num plyファイルのパラメータの数。// TODO#と空白で区切った数で入力しているので、もしheader部分がプロパティの個数と一緒の区切り方をしてたら変更する
 * @param loaded_point_data PointSetクラス
 */
void ObjectIO::load_ply_point_file(std::string file_name, std::string dir_path, int property_num, PointSet &loaded_point_data)
{
    std::fstream data_file;
    std::string file_path = dir_path + file_name;

    data_file.open(file_path, std::ios::in);

    // 文字列分割についてはここの手動用による文字列分割を参照
    //  https://marycore.jp/prog/cpp/std-string-split/

    std::string one_line_buffer;
    std::string separator = std::string(" ");
    auto separator_length = separator.length();

    // getlineで1行ずつ処理する
    while (std::getline(data_file, one_line_buffer))
    {
        // 1行をseparatorで区切ったものをリストとして保存
        std::vector<std::string> buf_list;
        auto offset = std::string::size_type(0);

        // 空白で区切って listに入れる処理
        while (1)
        {
            // offset以降で 区切り文字の出現位置を見つける
            // 見つからない場合std::string::nposを返す
            auto pos = one_line_buffer.find(separator, offset);

            // 見つからなかった場合は1行全部をリストに入れる
            if (pos == std::string::npos)
            {
                buf_list.push_back(one_line_buffer.substr(offset));
                break;
            }

            // 見つかった場合は区切り文字までをsubstrで指定することで 取り出してリストに入れる
            buf_list.push_back(one_line_buffer.substr(offset, pos - offset));
            // offsetを更新する
            offset = pos + separator_length;
        }

        // TODO 本当はplyファイルのheaderを認識して "end_header"から入力をしたい

        /**
         * @brief プロパティの数と一行をセパレートした値が一致するか
         * @param size  :vector,arrayのsize()
         * @param prop_num int パラメータのサイズ
         *
         */
        auto is_property_count_same = [](long unsigned int size, int prop_num)
        {
            if (int(size) == prop_num)
            {
                return true;
            }
            else
            {
                return false;
            }
        };

        /**
         * @brief 文字が"#"かどうか判定
         * @param buf_str std::string 判定する文字
         */
        auto is_first_sharp = [](std::string buf_str)
        {
            if (buf_str == "#")
            {
                return true;
            }
            else
            {
                return false;
            }
        };

        // 分割したものから xyzを取り出す。
        if (is_property_count_same(buf_list.size(), property_num))
        {
            int i = 0;
            // 最初
            if (!(is_first_sharp(buf_list.at(0))))
            {
                std::array<double, 3> vec_data;
                for (auto one_point_data : buf_list)
                {
                    try
                    {
                        // vec_data.push_back(std::stod(e));
                        vec_data[i++] = std::stod(one_point_data);
                    }
                    catch (const std::invalid_argument &e)
                    {
                        std::cout << "invalid argument" << std::endl;
                    }
                    catch (const std::out_of_range &e)
                    {
                        std::cout << "Out of range" << std::endl;
                    }
                }
                Eigen::Vector3d tmp = {vec_data.at(0), vec_data.at(1), vec_data.at(2)};
                loaded_point_data.add_point(tmp);
            }
        }
    }
}

std::array<double, 2> get_img_width_height(std::string img_path)
{
    // 画像サイズが欲しいので 取ってくる。;
    cv::Mat img = cv::imread(img_path);
    if (img.empty())
    {
        std::cout << "couldn't read the image." << std::endl;
    }

    std::cout << std::endl
              << "img_size:height width" << std::endl;
    std::cout << img.rows << " " << img.cols << std::endl;

    std::array<double, 2> img_size = {double(img.cols), double(img.rows)};
    return img_size;
}

/**
 * @brief 正距円筒から球の座標への変換
 * pointgetterからは、左上が原点の U,V座標で出力
 *
 */
Eigen::Vector3d equirectangular_to_sphere(double u, double v, double img_width, double img_height)
{
    // 正規化
    u /= img_width;
    v /= img_height;

    // 緯度経度計算
    double phi = u * 2 * M_PI;
    double theta = v * M_PI;

    // 方向ベクトル
    Eigen::Vector3d p = {abs(sin(theta)) * sin(phi), abs(sin(theta)) * cos(phi), cos(theta)};

    return p;
};

/**
 * @brief imgファイルからの対応点の読み込み。方向ベクトルの変換
 *
 * @param file_name
 * @param dir_path
 * @param img_path
 * @param loaded_point_data
 */
void ObjectIO::load_img_point_file(std::string file_name, std::string dir_path, std::string img_path, PointSet &loaded_point_data)
{
    std::fstream data_file;
    std::string file_path = dir_path + file_name;

    data_file.open(file_path, std::ios::in);

    std::string one_line_buffer;
    std::string separator = std::string(" ");
    auto separator_length = separator.length();

    std::array<double, 2> img_size = get_img_width_height(img_path);

    // getlineで1行ずつ処理する
    while (std::getline(data_file, one_line_buffer))
    {
        // 1行をseparatorで区切ったものをリストとして保存
        std::vector<std::string> buf_list;
        auto offset = std::string::size_type(0);

        // 空白で区切って listに入れる処理
        while (1)
        {
            // offset以降で 区切り文字の出現位置を見つける
            // 見つからない場合std::string::nposを返す
            auto pos = one_line_buffer.find(separator, offset);

            // 見つからなかった場合は1行全部をリストに入れる
            if (pos == std::string::npos)
            {
                buf_list.push_back(one_line_buffer.substr(offset));
                break;
            }

            // 見つかった場合は区切り文字までをsubstrで指定することで 取り出してリストに入れる
            buf_list.push_back(one_line_buffer.substr(offset, pos - offset));
            // offsetを更新する
            offset = pos + separator_length;
        }

        // TODO 本当はplyファイルのheaderを認識して "end_header"から入力をしたい

        /**
         * @brief プロパティの数と一行をセパレートした値が一致するか
         * @param size  :vector,arrayのsize()
         * @param prop_num int パラメータのサイズ
         *
         */
        auto is_property_count_same = [](long unsigned int size, int prop_num)
        {
            if (int(size) == prop_num)
            {
                return true;
            }
            else
            {
                return false;
            }
        };

        /**
         * @brief 文字が"#"かどうか判定
         * @param buf_str std::string 判定する文字
         */
        auto is_first_sharp = [](std::string buf_str)
        {
            if (buf_str == "#")
            {
                return true;
            }
            else
            {
                return false;
            }
        };

        // img ファイルは2
        int property_num = 2;

        // 分割したものから xyzを取り出す。
        if (is_property_count_same(buf_list.size(), property_num))
        {
            int i = 0;
            // 最初
            if (!(is_first_sharp(buf_list.at(0))))
            {
                std::array<double, 2> vec_data;
                for (auto one_point_data : buf_list)
                {
                    try
                    {
                        // vec_data.push_back(std::stod(e));
                        vec_data[i++] = std::stod(one_point_data);
                    }
                    catch (const std::invalid_argument &e)
                    {
                        std::cout << "invalid argument" << std::endl;
                    }
                    catch (const std::out_of_range &e)
                    {
                        std::cout << "Out of range" << std::endl;
                    }
                }
                loaded_point_data.add_point(equirectangular_to_sphere(vec_data.at(0), vec_data.at(1), img_size.at(0), img_size.at(1)));
            }
        }
    }
}

// void output_ply2(std::vector<Eigen::Vector3d> &point_data, std::string out_path, Eigen::Vector3d translation_t)
// {
//     // std::ios::app : 追記
//     // std::ios::out : 書き込み
//     std::ofstream output_ply(out_path, std::ios::out);

//     output_ply << "ply" << std::endl
//                << "format ascii 1.0" << std::endl
//                << "element vertex " << point_data.size() + 2 << std::endl
//                << "property float x" << std::endl
//                << "property float y" << std::endl
//                << "property float z" << std::endl
//                << "element edge 1" << std::endl
//                << "property int vertex1" << std::endl
//                << "property int vertex2" << std::endl
//                << "end_header" << std::endl
//                << "0 0 0" << std::endl
//                << translation_t(0) << " " << translation_t(0) << " " << translation_t(0) << std::endl;

//     for (const Eigen::Vector3d &tmp : point_data)
//     {
//         output_ply << tmp(0) << " " << tmp(1) << " " << tmp(2) << std::endl;
//     }

//     output_ply << "1 2" << std::endl;

//     // for (auto &tmp : point_data)
//     // {
//     //     std::cout << tmp(0) << " " << tmp(1) << " " << tmp(2) << std::endl;
//     // }
//     // std::cout << point_data.size() << std::endl;
// }
void ObjectIO::output_ply(PointSet &point_data, std::string out_path)
{
    // std::ios::app : 追記
    // std::ios::out : 書き込み
    std::ofstream output_ply(out_path, std::ios::out);

    output_ply << "ply" << std::endl
               << "format ascii 1.0" << std::endl
               << "element vertex " << point_data.get_point_num() << std::endl
               << "property float x" << std::endl
               << "property float y" << std::endl
               << "property float z" << std::endl
               << "end_header" << std::endl;

    for (const Eigen::Vector3d &tmp : point_data.get_point())
    {
        output_ply << tmp(0) << " " << tmp(1) << " " << tmp(2) << std::endl;
    }
}

void ObjectIO::print()
{
    std::cout << "operation load" << std::endl;
}