cmake_minimum_required(VERSION 3.5.1)

# CXXと指定すると C++のみのプロジェクトになり Cは考えない。
project(hello CXX)

option(OPEN3D_ENABLED "Use Open3D library" OFF)

add_executable(${PROJECT_NAME} src/hello.cpp)



# DEBUG mode のみに適用されるコンパイル定義
target_compile_definitions(hello
    PRIVATE
    $<$<CONFIG:DEBUG>:DEBUG>
)


# DEBUG mode のみに適用されるcompiler option
target_compile_options(hello
    PRIVATE
    $<$<CONFIG:DEBUG>:-Wall>
    $<$<CONFIG:DEBUG>:-g>
)





# やりたいことは、 debug時に commnet 欄を消すことと
# open3d を使うかどうかのオプションを追加するので、
# debug と open3d用の定義を追加してみよう

#
# set(CMAKE_CXX_STANDARD 20)
#
# option(USE_OPEN3D "Use Open3D library" OFF)
#
#
# find_package(OpenCV REQUIRED)
#
#
# if(USE_OPEN3D)
#     find_package(Open3D REQUIRED)
#     target_link_libraries(Open3D::Open3D)
# endif()
#
#
# # include dir
# include_directories(${PROJECT_SOURCE_DIR}/include)
# target_include_directories()
#
# # cppファイルを追加
# add_executable(
#     ${PROJECT_NAME}
#     src/main.cpp
#     src/pointset.cpp
#     src/operation.cpp
#     src/object_io.cpp
#     src/calc_pointset.cpp
#     src/capture_boxpoint.cpp
#     src/viewer.cpp
#     src/img_proc.cpp
# )
#
# # 実行ファイルをリンクする
# target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${rapidjson} Open3D::Open3D)
# # target_link_libraries(DisplayImage Eigen3::Eigen)
#
# # compiler option
# target_compile_options(${PROJECT_NAME} PUBLIC -g -Wvariadic-macros -Wswitch-enum -Wall -Wextra -pedantic -Wcast-align -Wcast-qual -Wconversion -Wdisabled-optimization -Wendif-labels -Wfloat-equal -Winit-self -Winline -Wlogical-op -Wmissing-include-dirs -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Wpacked -Wpointer-arith -Wredundant-decls -Wshadow -Wsign-promo -Wswitch-default -Wunsafe-loop-optimizations -Wwrite-strings)
#
# # c++20
# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
