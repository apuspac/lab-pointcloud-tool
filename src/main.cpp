#include "operation.hpp"
#include "pointset.hpp"
#include "calc_pointset.hpp"

int main(int argc, char *argv[])
{
    PointOperation opt;
    opt.option_process(argc, argv);

    opt.print();

    // ./Rotation -i image_point.dat ply_point.dat 1108_kyoiku.ply kyoiku.JPG ../../ply_data/test_idou/

    // PointSetIO::load_ply_file();

    // PointSet vec("test");
    // // vec.set_point()
    // // vec.print();

    // PointSetIO hello;
    // hello.print();

    // CalcPointSet world;
    // world.test_print();

    return 0;
}