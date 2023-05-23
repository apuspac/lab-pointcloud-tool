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
    opt.mode_select();

    // jsonファイル読みこみ
    FILE *fp = fopen("data/detections.json", "r");
    if (!fp)
    {
        std::cerr << "Failed to open the JSONFILE" << std::endl;

        return 1;
    }

    char readBuffer[65536];
    rapidjson::FileReadStream jsonfile(fp, readBuffer, std::size(readBuffer));

    // パース
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

    // 配列で読み込む場合これ
    const rapidjson::Value &merge_data = doc["merged_data"].GetArray();

    for (const auto &detect_img : merge_data.GetArray())
    {
        // std::cout << detect_img["file_name"].GetString() << std::endl;
        const rapidjson::Value &bbox_info = detect_img["bbox_info"].GetArray();

        for (const auto &bbox : bbox_info.GetArray())
        {
            // std::cout << bbox["xmin"].GetDouble() << std::endl;
        }
    }

    // std::cout << doc["xmin"].GetDouble() << std::endl;
    // std::cout << doc["ymin"].GetDouble() << std::endl;
    // std::cout << doc["class"].GetInt() << std::endl;

    fclose(fp);

    return 0;
}