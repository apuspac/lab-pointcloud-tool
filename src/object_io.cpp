/**
 * @file object_io.cpp
 * @brief plyファイル 点データのIOの実装
 */
#include "object_io.hpp"

/**
 * @brief オプション処理
 *
 * @param argc main()の引数のargcを指定
 * @param argv main()の引数のcharを指定
 * @param operation PointOperationクラスの参照を指定。オプション処理により取得したファイル名などを格納するため
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
    const char *opt_string = "m:x:p:d:i:h:o";

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
            {"mode", required_argument, 0, 'o'},
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
            operation.set_corresp_img_file_name(std::string(optarg));
            break;

        case 'x':
            operation.set_corresp_ply_file_name(std::string(optarg));
            break;
            ;
        case 'p':
            operation.set_plyfile_name(std::string(optarg));
            break;

        case 'i':
            operation.set_img_file_path(std::string(optarg));
            break;

        case 'd':
            operation.set_default_dir_path(std::string(optarg));
            break;
        case 'o':
            operation.set_mode(std::string(optarg));
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
 * @brief プロパティの数と一行をセパレートした数が一致するか
 * @param size vector,arrayのsize()
 * @param prop_num パラメータのサイズ
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
 * @param buf_str 判定する文字
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

/**
 * @brief plyファイルを読み込む
 *
 * 空白で区切った数で点データかどうか判定をしているので、もしheader部分で、プロパティの個数と一緒のプロパティ数があれば、変更する
 *
 *
 * @param file_name plyファイル名
 * @param dir_path plyファイルのあるディレクトリ
 * @param property_num plyファイルのパラメータの数。
 * @param loaded_point_data 格納するPointSetクラス
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

        // TODO plyファイルのheaderを認識して "end_header"の次の行から入力をしたい

        // 分割したものから xyzを取り出す。
        if (is_property_count_same(buf_list.size(), property_num))
        {
            int i = 0;

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

/**
 * @brief Get the img width height
 *
 * @param img_path 画像の相対パス
 * @return std::array<double, 2> 画像の幅, 画像の高さの順に取得
 */
std::array<double, 2> get_img_width_height(std::string img_path)
{
    // 画像を取得
    cv::Mat img = cv::imread(img_path);
    if (img.empty())
    {
        std::cout << "couldn't read the image." << std::endl;
    }

    // サイズを調べ格納。
    std::array<double, 2> img_size = {double(img.cols), double(img.rows)};

    std::cout << std::endl
              << "img_size:height width" << std::endl;
    std::cout << img.rows << " " << img.cols << std::endl;

    return img_size;
}

/**
 * @brief 正距円筒図法から球の座標への変換
 * 左上が原点の U,V座標で出力されてる前提
 *
 */

/**
 * @brief 正距円筒図法(360度画像)の画素値から単位球に投影したとき方向ベクトルへ変換する。左上が原点でのuv座標形式の画素値が入力される前提。緯度経度を計算し、xyzに変換する。
 *
 * @param u 画素値の横座標
 * @param v 画素値の縦座標
 * @param img_width 360度画像の横
 * @param img_height 360度画像の縦
 * @return Eigen::Vector3d 変換したベクトルを返す
 */
Eigen::Vector3d equirectangular_to_sphere(double u, double v, double img_width, double img_height)
{
    // 正規化
    u /= img_width;
    v /= img_height;

    // 緯度経度計算
    double phi = u * 2 * M_PI;
    double theta = v * M_PI;

    // 方向ベクトルに変換
    Eigen::Vector3d p = {abs(sin(theta)) * sin(phi), abs(sin(theta)) * cos(phi), cos(theta)};

    return p;
}

/**
 * @brief imgファイルから点の読み込み。方向ベクトルの変換処理も一緒に行う
 *
 * @param file_name img点のファイル名
 * @param dir_path img点のファイルがあるディレクトリパス
 * @param img_path 画像のファイルパス
 * @param loaded_point_data 取得したpointを格納するPointset
 */
void ObjectIO::load_img_point_file(std::string file_name, std::string dir_path, std::string img_path, PointSet &loaded_point_data)
{
    std::fstream data_file;
    std::string file_path = dir_path + file_name;

    data_file.open(file_path, std::ios::in);

    std::string one_line_buffer;
    std::string separator = std::string(" ");
    auto separator_length = separator.length();

    // 画像の縦横サイズを取得
    std::array<double, 2> img_size = get_img_width_height(img_path);

    // 点群の読み込みとほぼ同じことをしているので、 ほんとは一緒にしたいが、PointSetを画像点も扱えるようにしないといけない。

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

        // img ファイルは2固定
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
                // 変換する処理をはさんで格納
                loaded_point_data.add_point(equirectangular_to_sphere(vec_data.at(0), vec_data.at(1), img_size.at(0), img_size.at(1)));
            }
        }
    }
}

/**
 * @brief PointSetをplyファイルに書き込み、出力する
 *
 * @param point_data 出力するpointsetオブジェクト
 * @param out_path 出力する場所のパス
 */
void ObjectIO::output_ply(PointSet &point_data, std::string out_path)
{
    // std::ios::app : 追記
    // std::ios::out : 書き込み
    std::ofstream output_ply(out_path, std::ios::out);

    if (point_data.get_edge_num() > 0)
    {
        // edgeがある場合
        output_ply << "ply" << std::endl
                   << "format ascii 1.0" << std::endl
                   << "element vertex " << point_data.get_point_num() << std::endl
                   << "property float x" << std::endl
                   << "property float y" << std::endl
                   << "property float z" << std::endl
                   << "element edge " << point_data.get_edge_num() << std::endl
                   << "property int vertex1" << std::endl
                   << "property int vertex2" << std::endl
                   << "end_header" << std::endl;

        for (const Eigen::Vector3d &tmp : point_data.get_point_all())
        {
            output_ply << tmp(0) << " " << tmp(1) << " " << tmp(2) << std::endl;
        }

        for (const auto &tmp : point_data.get_edge_all())
        {
            output_ply << tmp.at(0) << " " << tmp.at(1) << std::endl;
        }
    }
    else
    {
        // edgeがない場合
        output_ply << "ply" << std::endl
                   << "format ascii 1.0" << std::endl
                   << "element vertex " << point_data.get_point_num() << std::endl
                   << "property float x" << std::endl
                   << "property float y" << std::endl
                   << "property float z" << std::endl
                   << "end_header" << std::endl;

        for (const Eigen::Vector3d &tmp : point_data.get_point_all())
        {
            output_ply << tmp(0) << " " << tmp(1) << " " << tmp(2) << std::endl;
        }
    }
}

/**
 * @brief 原点から距離を伸ばした位置の点を計算
 * 極座標変換をし、距離rから 伸ばした距離の位置を計算。
 *
 * @param point 計算する点
 * @param range 原点からの距離
 * @return Eigen::Vector3d
 */
Eigen::Vector3d ObjectIO::extend_distance_from_point_and_origin(Eigen::Vector3d point, double range)
{
    Eigen::Vector3d tmp_normalize = point.normalized();

    double theta = std::acos(
        tmp_normalize(2) /
        std::sqrt(std::pow(tmp_normalize(0), 2.0) + std::pow(tmp_normalize(1), 2.0) + std::pow(tmp_normalize(2), 2.0)));
    double phi = std::atan2(tmp_normalize(1), tmp_normalize(0));
    double r = range;

    Eigen::Vector3d extend_point = {r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta)};

    return extend_point;
}
