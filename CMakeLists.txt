# CMakeのバージョン指定
cmake_minimum_required(VERSION 3.5.1)

# # CXXと指定すると C++のみのプロジェクトになり Cは考えない。
project(Rotation CXX)

find_package(OpenCV REQUIRED)


option(OPEN3D_ENABLED "Use Open3D library" FALSE)
option(MATPLOTLIB_ENABLED "Use python matplotlib" FALSE)

# -DOPEN3D_ENABLED=ONで有効
if(OPEN3D_ENABLED)                # これはgenerator expression使えないらしい。
    find_package(Open3D REQUIRED)
endif()

if(MATPLOTLIB_ENABLED)
    find_package(Python3 COMPONENTS Development NumPy)
endif()



# cppファイルを追加
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/pointset.cpp
    src/operation.cpp
    src/object_io.cpp
    src/calc_pointset.cpp
    src/capture_boxpoint.cpp
    src/viewer.cpp
    src/img_proc.cpp
)

# include dir
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        $<$<BOOL:${MATPLOTLIB_ENABLED}>:/usr/local/include/matplotlib-cpp>
        $<$<BOOL:${MATPLOTLIB_ENABLED}>:${Python3_INCLUDE_DIRS}>
        $<$<BOOL:${MATPLOTLIB_ENABLED}>:${Python3_NumPy_INCLUDE_DIRS}>
)

# c++20
target_compile_features(${PROJECT_NAME} 
    PRIVATE 
        cxx_std_20
)

# compile option
target_compile_options(${PROJECT_NAME}
    PRIVATE
    -g -Wvariadic-macros -Wswitch-enum -Wall -Wextra -pedantic -Wcast-align -Wcast-qual -Wconversion -Wdisabled-optimization -Wendif-labels -Wfloat-equal -Winit-self -Winline -Wlogical-op -Wmissing-include-dirs -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Wpacked -Wpointer-arith -Wredundant-decls -Wshadow -Wsign-promo -Wswitch-default -Wunsafe-loop-optimizations -Wwrite-strings
    PRIVATE
        $<$<CONFIG:DEBUG>:-g>
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        $<$<CONFIG:DEBUG>:DEBUG>          # DEBUG mode のみに適用されるコンパイル定義
        $<$<BOOL:${OPEN3D_ENABLED}>:OPEN3D_ENABLED>
        $<$<BOOL:${MATPLOTLIB_ENABLED}>:MATPLOTLIB_ENABLED>
)


target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_LIBS}
        ${rapidjson}
        # ${matplotlibcpp}
        $<$<BOOL:${OPEN3D_ENABLED}>:Open3D::Open3D>  # OPEN3D_REQUIRED
        # $<$<BOOL:${MATPLOTLIB_ENABLED}>:matplotlib-cpp>
        $<$<BOOL:${MATPLOTLIB_ENABLED}>:Python3::Python>
        $<$<BOOL:${MATPLOTLIB_ENABLED}>:Python3::NumPy>
)
# target_link_libraries(DisplayImage Eigen3::Eigen)
