project(example_matplotlibcpp)

cmake_minimum_required(VERSION 2.8)

find_package(Python3 COMPONENTS Interpreter Development NumPy)

include_directories(${Python3_INCLUDE_DIRS} include)

add_executable(result_plot src/main.cpp)

target_link_libraries(result_plot ${Python3_LIBRARIES})
