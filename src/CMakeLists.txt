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
    calc_pointset.cpp
)

# ここeigenはなんでリンクしなくていいのかよくわからん
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
# target_link_libraries(DisplayImage Eigen3::Eigen)

# compiler option
target_compile_options(${PROJECT_NAME} PUBLIC -Wall --pedantic-errors)

# c++の標準規格は指定しなかったらいくつになるんですかね
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)