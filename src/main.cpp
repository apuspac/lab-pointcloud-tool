#include "operation.hpp"
#include "pointset.hpp"
#include "calc_pointset.hpp"
#include "object_io.hpp"

#include "rapidjson/document.h"

int main(int argc, char *argv[])
{
    // ./Rotation --img_cp img.dat --ply_cp ply.dat --ply kyoiku.ply --img ../../img/kyoiku.JPG --dir ../../ply_data/test_idou/

    // 有効桁数
    std::cout << std::setprecision(15);

    PointOperation opt;
    ObjectIO::option_process(argc, argv, opt);
    opt.print();
    opt.mode_select();

    // std::string bbox_info = "{\"xmin\":1583.7875976562,"
    //                         "\"ymin\":1327.7779541016,"
    //                         "\"xmax\":1835.1019287109,"
    //                         "\"ymax\":1665.6372070312,"
    //                         "\"confidence\":0.7136921287,"
    //                         "\"class\":9,"
    //                         "\"name\":\"square_box_w_marker_front_side\"}";

    // rapidjson::Document doc;
    // doc.Parse(bbox_info.c_str());

    // std::cout << doc["xmin"].GetDouble() << std::endl;
    // std::cout << doc["ymin"].GetDouble() << std::endl;
    // std::cout << doc["name"].GetString() << std::endl;
    // std::cout << doc["class"].GetInt() << std::endl;

    return 0;
}