#include "operation.hpp"

int PointOperation::option_process(int argc, char **argv)
{
    std::cout << argc << std::endl;

    for (int i = 0; i < argc; i++)
    {
        std::cout << argv[i] << std::endl;
    }

    return 0;
}

void PointSetIO::print()
{
    std::cout << "operation load" << std::endl;
}