#include "operation.hpp"
#include "pointset.hpp"
#include "calc_pointset.hpp"
#include "object_io.hpp"

int main(int argc, char *argv[])
{
    // ./Rotation --img_cp img.dat --ply_cp ply.dat --ply kyoiku.ply --img ../../img/kyoiku.JPG --dir ../../ply_data/test_idou/

    // 有効桁数
    std::cout << std::setprecision(15);
    PointOperation opt;
    ObjectIO::option_process(argc, argv, opt);
    opt.print();
    opt.mode_select();
    // opt.transform_rotate();
    // opt.Rotation_point();
    // opt.Rotation_point_simulation();
    // opt.transform_rotate_simulation();
    // opt.capture_boxpoint();
    // opt.capture_segmentation_point();

    return 0;
}