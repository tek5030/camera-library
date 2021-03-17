cmake_minimum_required(VERSION 3.10)
project(example CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

find_package(tek5030 CONFIG REQUIRED)

add_executable(rs-single-example main-realsense-single.cpp)
add_executable(rs-stereo-example main-realsense-stereo.cpp)
add_executable(rs-rgbd-example main-realsense-rgbd.cpp)
add_executable(cv-example main-cvcam.cpp)
add_executable(dual-example main-dualcam.cpp)

target_link_libraries(rs-single-example PRIVATE tek5030::tek5030)
target_link_libraries(rs-stereo-example PRIVATE tek5030::tek5030)
target_link_libraries(rs-rgbd-example   PRIVATE tek5030::tek5030)
target_link_libraries(cv-example        PRIVATE tek5030::tek5030)
target_link_libraries(dual-example      PRIVATE tek5030::tek5030)
