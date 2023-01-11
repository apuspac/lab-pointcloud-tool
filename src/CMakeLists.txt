# CMakeのバージョン指定
cmake_minimum_required(VERSION 3.1)
# project name
project(Rotation)

find_package(OpenCV REQUIRED)
# find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)

# cppファイルを追加
add_executable(
    ${PROJECT_NAME}
    main.cpp
    pointset.cpp
    operation.cpp
    object_io.cpp
    calc_pointset.cpp
    capture_boxpoint.cpp
)

# ここeigenはなんでリンクしなくていいのかよくわからん
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
# target_link_libraries(DisplayImage Eigen3::Eigen)

# compiler option
# target_compile_options(${PROJECT_NAME} PUBLIC -Wall --pedantic-errors)
target_compile_options(${PROJECT_NAME} PUBLIC -g -Wall -Wextra -pedantic -Wcast-align -Wcast-qual -Wconversion -Wdisabled-optimization -Wendif-labels -Wfloat-equal -Winit-self -Winline -Wlogical-op -Wmissing-include-dirs -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Wpacked -Wpointer-arith -Wredundant-decls -Wshadow -Wsign-promo -Wswitch-default -Wswitch-enum -Wunsafe-loop-optimizations -Wvariadic-macros -Wwrite-strings)


# c++の標準規格は指定しなかったらいくつになるんですかね
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)