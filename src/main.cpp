#include "operation.hpp"
#include "pointset.hpp"
#include "calc_pointset.hpp"
#include "object_io.hpp"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <cstdio>

int main(int argc, char *argv[])
{
    // ./Rotation --img_cp img.dat --ply_cp ply.dat --ply kyoiku.ply --img ../../img/kyoiku.JPG --dir ../../ply_data/test_idou/

    // 有効桁数
    std::cout << std::setprecision(15);

    PointOperation opt;
    ObjectIO::option_process(argc, argv, opt);
    opt.print();
    // opt.mode_select();

    // jsonファイル読みこみ
    const std::string jsonFilePath = "../../data/detection.json";

    std::ifstream jsonFile(jsonFilePath);
    if (!jsonFile.is_open())
    {
        std::cerr << "Failed to open the jsonfile" << std::endl;
        return 1;
    }

    // ファイル内容を文字列として読み込む
    std::string jsonContent(
        std::istreambuf_iterator<char>(jsonFile),
        std::istreambuf_iterator<char>());

    // jsonファイルを閉じる
    jsonFile.close();

    // パース
    rapidjson::Document doc;
    doc.Parse(jsonContent.c_str());

    // errcheck
    if (doc.HasParseError())
    {
        std::cerr << "Failed to parse the JSON content." << std::endl;
        return 1;
    }

    // std::cout << doc["xmin"].GetDouble() << std::endl;
    // std::cout << doc["ymin"].GetDouble() << std::endl;
    std::cout << doc["inputpath"].GetString() << std::endl;
    // std::cout << doc["class"].GetInt() << std::endl;

    return 0;
}