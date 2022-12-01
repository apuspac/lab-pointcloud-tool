#include "operation.hpp"

/**
 * @brief オプション処理
 *
 * @param argc int main()の引数のargc
 * @param argv char* main()の引数のargv
 * @return int 正常終了したか TODO:チェックを作っていない
 */
int PointOperation::option_process(int argc, char **argv)
{

    // m: imgの対応点 .dat
    // x: plyの対応点 .dat
    // p: plyファイル .ply
    // i: imgファイル .jpg,png
    // d: デフォルトdirpath
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
            corresp_img_file_name.push_back(std::string(optarg));
            break;

        case 'x':
            corresp_ply_file_name.push_back(std::string(optarg));
            break;
            ;
        case 'p':
            ply_file_path.push_back(std::string(optarg));
            break;

        case 'i':
            img_file_path.push_back(std::string(optarg));
            break;

        case 'd':
            default_dir_path = std::string(optarg);
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

    return 0;
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
    for (auto tmp : ply_file_path)
    {
        std::cout << tmp << std::endl;
    }
}

void PointSetIO::load_ply_file(std::string file_name, std::string dir_path, int property_num, PointSet *loaded_point_data)
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

        // 分割したものから xyzを取り出す。
        // プロパティの数があっているか？
        // auto is_property_count_same = (){};

        if (buf_list.size() == property_num)
        {
            // 最初
            if (buf_list.at(0) != "#")
            {
                std::vector<double> vec_data;
                for (auto e : buf_list)
                {
                    try
                    {
                        vec_data.push_back(std::stod(e));
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
                // point_data.push_back(tmp);
            }
        }
    }
}

void PointSetIO::print()
{
    std::cout << "operation load" << std::endl;
}