cmake_minimum_required(VERSION 3.11)

project(Test)

add_executable(test ekf_state_estimation.cpp)

target_include_directories(test PUBLIC ${CMAKE_SOURCE_DIR}/include /opt/ros/noetic/include)
