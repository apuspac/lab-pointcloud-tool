#include "operation.hpp"
#include "pointset.hpp"
#include "calc_pointset.hpp"
#include "object_io.hpp"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <cstdio>

#ifdef OPEN3D_ENABLED
#include "open3d/Open3D.h"
#endif



int main(int argc, char *argv[])
{
#ifdef DEBUG
    std::cout << "DEBUG_MODE" << std::endl;
#endif

#ifdef OPEN3D_ENABLED
    std::cout << "USE_OPEN3D" << std::endl;
#endif

#ifdef MATPLOTLIB_ENABLED
    std::cout << "USE_MATPLOTLIB" << std::endl;
#endif

    std::cout << std::setprecision(15);

    PointOperation opt;
    ObjectIO::option_process(argc, argv, opt);
    opt.print();
    opt.mode_select();

    return 0;
}
