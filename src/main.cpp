#include "operation.hpp"
#include "pointset.hpp"
#include "calc_pointset.hpp"
#include "object_io.hpp"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <cstdio>

#include "open3d/Open3D.h"

int main(int argc, char *argv[])
{

    // 有効桁数
    std::cout << std::setprecision(15);

    PointOperation opt;
    ObjectIO::option_process(argc, argv, opt);
    opt.print();
    opt.mode_select();


#ifdef DEBUG
    std::cout << "DEBUG_MODE____" << std::endl;
#endif

#ifdef OPEN3D_ENABLED
    std::cout << "USE_OPEN3D___" << std::endl;
#endif

    return 0;
}
