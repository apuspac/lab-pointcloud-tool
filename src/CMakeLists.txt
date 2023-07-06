# CMakeのバージョン指定
cmake_minimum_required(VERSION 3.1)

# project name
project(Rotation)

find_package(OpenCV REQUIRED)

# cmake_policy(SET CMP0074 NEW)
# set(Open3D_DIR "${HOME}/open3d_install")
find_package(Open3D REQUIRED)

# cppファイルを追加
add_executable(
    ${PROJECT_NAME}
    main.cpp
    pointset.cpp
    operation.cpp
    object_io.cpp
    calc_pointset.cpp
    capture_boxpoint.cpp
    viewer.cpp
)

# ここeigenはなんでリンクしなくていいのかよくわからん
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${rapidjson} Open3D::Open3D)

# target_link_libraries(DisplayImage Eigen3::Eigen)

# compiler option
# target_compile_options(${PROJECT_NAME} PUBLIC -Wall --pedantic-errors)
target_compile_options(${PROJECT_NAME} PUBLIC -g -Wvariadic-macros -Wswitch-enum -Wall -Wextra -pedantic -Wcast-align -Wcast-qual -Wconversion -Wdisabled-optimization -Wendif-labels -Wfloat-equal -Winit-self -Winline -Wlogical-op -Wmissing-include-dirs -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Wpacked -Wpointer-arith -Wredundant-decls -Wshadow -Wsign-promo -Wswitch-default -Wunsafe-loop-optimizations -Wwrite-strings)

# c++の標準規格は指定しなかったらいくつになるんですかね
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)