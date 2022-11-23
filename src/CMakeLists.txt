# CMakeのバージョン指定
cmake_minimum_required(VERSION 3.1)
project(Rotation)

find_package(OpenCV REQUIRED)

# find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)
add_executable(Rotation main.cpp)
target_link_libraries(Rotation ${OpenCV_LIBS})

# target_link_libraries(DisplayImage Eigen3::Eigen)