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
    // j: jsonファイル .json
    // d: output dir path
    // o: mode 選択
    // h: help
    // 複数入力をさせたいけど ちょっと面倒っぽいので、 2回指定するときは、2回とも入力する。
    const char *opt_string = "m:x:p:d:j:i:h:o";

    // オプション処理のlong用
    // 長い文字列-> 一文字(opt_string)ように変えてる。
    // {*name,  has_arg,    *flag,  val},
    // has_argは 引数が必要なら has_arg, 必要なかったらno_arg どっちでもよいときはoptional_arg
    // flag 0 のときは valが返される
    static struct option long_options[] =
        {
            {"img_cp", required_argument, 0, 'm'},
            {"ply_cp", required_argument, 0, 'x'},
            {"ply", required_argument, 0, 'p'},
            {"img", required_argument, 0, 'i'},
            {"json", required_argument, 0, 'j'},
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

        case 'j':
            operation.set_json_path(std::string(optarg));
            break;

        case 'd':
            operation.set_output_dir_path(std::string(optarg));
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
                      << "--dir,      -d : output dir path" << std::endl;
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
 * @param file_path plyファイル名
 * @param dir_path plyファイルのあるディレクトリ
 * @param property_num plyファイルのパラメータの数。
 * @param loaded_point_data 格納するPointSetクラス
 */
void ObjectIO::load_ply_point_file(PointSet &loaded_point_data, std::string file_path=""  )
{
    std::fstream data_file;

    data_file.open(file_path, std::ios::in);

    // 文字列分割についてはここの手動用による文字列分割を参照
    //  https://marycore.jp/prog/cpp/std-string-split/

    std::string one_line_buffer;
    std::string separator = std::string(" ");
    auto separator_length = separator.length();

    int property_num = 0;

    // 最初に end_headerまでを処理する
    while (std::getline(data_file, one_line_buffer))
    {

        std::vector<std::string> buf_list;
        auto offset = std::string::size_type(0);

        while (1)
        {
            auto pos = one_line_buffer.find(separator, offset);
            if (pos == std::string::npos)
            {
                buf_list.push_back(one_line_buffer.substr(offset));
                break;
            }

            else{
                buf_list.push_back(one_line_buffer.substr(offset, pos - offset));
                offset = pos + separator_length;
            }

        }

        if (one_line_buffer == "end_header") break;

        // # はコメントなので無視
        if (!(is_first_sharp(buf_list.at(0))))
        {
            if(buf_list.at(0) == "property"){
                property_num++;
            }
        }
    }


    std::cout << "property_num_count:" << property_num << std::endl;

    // end_headerまで読み込んだので、次から点データを読み込む
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

        // 分割したものから xyzを取り出す。
        if (is_property_count_same(buf_list.size(), property_num))
        {
            int i = 0;
            bool not_string_flag = true;

            if (!(is_first_sharp(buf_list.at(0))))
            {
                std::array<double, 3> vec_data;
                for (auto one_point_data : buf_list)
                {
                    double num;

                    try
                    {
                        // vec_data.push_back(std::stod(e));
                        num = std::stod(one_point_data);
                    }
                    catch (const std::invalid_argument &e)
                    {
                        std::cout << "invalid argument:" << one_point_data << std::endl;
                        not_string_flag = false;
                        std::cout << one_line_buffer << std::endl;
                    }
                    catch (const std::out_of_range &e)
                    {
                        std::cout << "Out of range" << std::endl;
                        not_string_flag = false;
                    }

                    // std::cout << not_string_flag << std::endl;
                    if (not_string_flag == true)
                    {
                        vec_data[i++] = num;
                    }
                }

                if (not_string_flag == true)
                {
                    Eigen::Vector3d tmp = {vec_data.at(0), vec_data.at(1), vec_data.at(2)};
                    loaded_point_data.add_point(tmp);
                }
            }
        }
    }
}

/**
 * @brief Get img width height
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
 * @param img_path 画像のファイルパス
 * @param loaded_point_data 取得したpointを格納するPointset
 * @param file_path img点のファイル名
 */
void ObjectIO::load_img_point_file(std::string img_path, PointSet &loaded_point_data, std::string file_path = "")
{
    std::fstream data_file;

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

    if (point_data.is_empty_edge() == false)
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
    if (point_data.is_empty() == false)
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

int ObjectIO::load_detection_json_file(std::string filepath, DetectionData &detect, std::string img_path)
{
    std::cout << "load_detection_jsonfile" << std::endl;
    std::cout << "filename: " << filepath << std::endl;

    // 画像の縦横サイズを取得
    std::array<double, 2> img_size = get_img_width_height(img_path);

    // jsonファイル読み込み
    FILE *fp = fopen(filepath.c_str(), "r");
    if (!fp)
    {
        std::cerr << "Failed to open the JSONFILE" << std::endl;
        fclose(fp);
        return 1;
    }

    char readBuffer[65536];
    rapidjson::FileReadStream jsonfile(fp, readBuffer, std::size(readBuffer));

    // parse
    rapidjson::Document doc;
    doc.ParseStream(jsonfile);

    // errcheck
    if (doc.HasParseError())
    {
        std::cerr << "Failed to parse the JSON content." << std::endl;
        return 1;
    }

    // 読み込むときはこれ
    std::cout << doc["input_path"].GetString() << std::endl;

    // 配列がある場合は getArray
    const rapidjson::Value &merge_data = doc["merged_data"].GetArray();

    for (const auto &detect_img : merge_data.GetArray())
    {
        BBox_OneIMG bboxdata;
        Mask_OneIMG maskdata;

        // file_name
        bboxdata.set_img_name(detect_img["file_name"].GetString());
        maskdata.set_img_name(detect_img["file_name"].GetString());

        // bbox set
        if (detect_img.HasMember("bbox_info"))
        {
            const rapidjson::Value &bbox_info = detect_img["bbox_info"];

            for (const auto &bbox : bbox_info.GetArray())
            {
                BBox bbox_tmp(
                    bbox["xmin"].GetDouble(),
                    bbox["ymin"].GetDouble(),
                    bbox["xmax"].GetDouble(),
                    bbox["ymax"].GetDouble());
                bbox_tmp.set_class_name(bbox["name"].GetString());
                bbox_tmp.set_class_num(bbox["class"].GetInt());
                bbox_tmp.set_confidence(bbox["confidence"].GetDouble());

                bbox_tmp.equirectangular_to_sphere(img_size.at(0), img_size.at(1));
                bboxdata.add_bbox(bbox_tmp);
            }

            detect.set_bbox_data(bboxdata);
        }

        // mask set
        if (detect_img.HasMember("mask_info"))
        {
            const rapidjson::Value &mask_info = detect_img["mask_info"];
            for (const auto &mask : mask_info.GetArray())
            {
                Mask mask_data;

                mask_data.set_class_name(mask["class_name"].GetString());
                const rapidjson::Value &one_mask = mask["contour"];

                // bboxと違って ちょっと入れ子になるので注意。
                for (const auto &pixel : one_mask.GetArray())
                {
                    const rapidjson::Value &pixel_mask = pixel;

                    for (const auto &pixel_tmp : pixel_mask.GetArray())
                    {
                        // intかdoubleか判定して doubleはcastして読み込み
                        int tmp_zero = pixel_tmp[0].IsInt() ? pixel_tmp[0].GetInt() : static_cast<int>(pixel_tmp[0].GetDouble());
                        int tmp_one = pixel_tmp[1].IsInt() ? pixel_tmp[1].GetInt() : static_cast<int>(pixel_tmp[1].GetDouble());
                        mask_data.add_mask(tmp_zero, tmp_one);
                    }
                }
                mask_data.equirectangular_to_sphere(img_size.at(0), img_size.at(1));
                maskdata.set_mask_data(mask_data);
            }

            detect.set_mask_data(maskdata);
        }
    }

    fclose(fp);

    return 0;
}

void ObjectIO::output_csv_2double(std::string filename, std::vector<std::vector<double>> csv_data)
{
    std::ofstream outputFile(filename);

    if (!outputFile.is_open())
    {
        std::cout << "failed to open file" << std::endl;
    }

    for (const auto &row : csv_data)
    {
        for (size_t i = 0; i < row.size(); ++i)
        {
            outputFile << row[i];

            // 最後の列でない場合はカンマを追加
            if (i < row.size() - 1)
            {
                outputFile << ",";
            }
        }
        outputFile << std::endl; // 行の終わり
    }

    std::cout << "output csv file complete: " << filename << std::endl;
}

void ObjectIO::create_dir(std::string dir_path)
{
    bool result = std::filesystem::create_directories(dir_path);

    assert(result);
}


// 結局これ以外のoutput_datは、一つの用途に特化してるので、
// 関数名を変えて、何をoutputしようとしてるかを明示するようにしたほうが良かったな。
void ObjectIO::output_dat(std::string filename, std::vector<std::string> s_data)
{
    std::ofstream outputFile(filename);

    if(!outputFile.is_open()){
        std::cout << "failed to open file" << std::endl;
    }

    for( const auto &p : s_data){
        outputFile << p << std::endl;
    }

    std::cout << "output string file complete: " << filename << std::endl;
}

void ObjectIO::output_dat(std::string filename, std::vector<Eigen::Vector3d> p_data)
{
    std::ofstream outputFile(filename);

    if (!outputFile.is_open())
    {
        std::cout << "failed to open file" << std::endl;
    }
    outputFile << p_data.size() << std::endl;

    for (const auto &p : p_data)
    {
        outputFile << p(0) << " " << p(1) << " " << p(2) << std::endl;
    }

    std::cout << "output point file complete: " << filename << std::endl;
}

void ObjectIO::output_dat(std::string filename, std::vector<std::pair<int, int>> i_data)
{

    std::ofstream outputFile(filename);

    if (!outputFile.is_open())
    {
        std::cout << "failed to open file" << std::endl;
    }

    outputFile << i_data.size() << std::endl;

    for (const auto &p : i_data)
    {
        outputFile << p.first << " " << p.second << std::endl;
    }

    std::cout << "output pixel file complete: " << filename << std::endl;

}

void ObjectIO::output_dat(std::string filename, std::vector<double> i_data)
{

    std::ofstream outputFile(filename);

    if (!outputFile.is_open())
    {
        std::cout << "failed to open file" << std::endl;
    }

    outputFile << i_data.size() << std::endl;

    for (const auto &p : i_data)
    {
        outputFile << p << std::endl;
    }

    std::cout << "output pixel file complete: " << filename << std::endl;
}


void ObjectIO::output_dat(std::string filename, std::vector<int> i_data)
{

    std::ofstream outputFile(filename);

    if (!outputFile.is_open())
    {
        std::cout << "failed to open file" << std::endl;
    }

    outputFile << i_data.size() << std::endl;

    for (const auto &p : i_data)
    {
        outputFile << p << std::endl;
    }

    std::cout << "output pixel file complete: " << filename << std::endl;
}
