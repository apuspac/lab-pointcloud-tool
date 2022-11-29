#include "operation.hpp"
#include "pointset.hpp"
#include "calc_pointset.hpp"

int main(int argc, char *argv[])
{
    PointOperation opt;
    opt.option_process(argc, argv);

    std::cout
        << argc << std::endl;

    // for (int i = 0; i < argc; i++)
    // {
    //     std::cout << argv[i] << std::endl;
    // }

    PointSet vec("test");
    // vec.set_point()
    // vec.print();

    PointSetIO hello;
    hello.print();

    CalcPointSet world;
    world.test_print();

    return 0;
}