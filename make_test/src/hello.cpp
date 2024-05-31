#include <iostream>

// #if !defined(BUILD_TYPE)
//     #define BUILD_TYPE "<None>"
// #endif

int main() {

// 何も指定してないと 何もでない。
#ifdef DEBUG
    std::cout << "Hello, World! Build type: " << "debug" << std::endl;
#endif // DEBUG


#ifdef USE_OPEN3D
    std::cout << "Hello, World! Build type: " << "OPEN3D" << std::endl;
#endif // USE_OPEN3D

    return 0;
}
