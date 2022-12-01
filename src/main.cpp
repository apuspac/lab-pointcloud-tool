#include "operation.hpp"
#include "pointset.hpp"
#include "calc_pointset.hpp"
#include "object_io.hpp"

int main(int argc, char *argv[])
{
    // ./Rotation --img_cp img.dat --ply_cp ply.dat --ply kyoiku.ply --img ../../img/kyoiku.JPG --dir ../../ply_data/test_idou/

    //有効桁数
    std::cout << std::setprecision(15);
    PointOperation opt;
    ObjectIO::option_process(argc, argv, opt);
    opt.print();

    // opt.transform_coordinate();
    // opt.Rotation_point();
    opt.Rotation_point_simlation();

    return 0;
}