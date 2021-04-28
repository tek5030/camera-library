cmake_minimum_required(VERSION 3.10)
project(example CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# Uncomment if you use this file as a template for a new project
#find_package(tek5030 CONFIG OPTIONAL_COMPONENTS rs2)

add_executable(cv-example main-cvcam.cpp)
add_executable(dual-example main-dualcam.cpp)
add_executable(kitti-example main-kitticam.cpp)

target_link_libraries(cv-example    PRIVATE tek5030::tek5030)
target_link_libraries(dual-example  PRIVATE tek5030::tek5030)
target_link_libraries(kitti-example PRIVATE tek5030::tek5030)

if (NOT TARGET tek5030::rs2)
  message (WARNING "library tek5030 is built without support for realsense2.\nSkipping realsense examples.")
else()
  add_executable(rs-single-example main-realsense-single.cpp)
  add_executable(rs-stereo-example main-realsense-stereo.cpp)
  add_executable(rs-rgbd-example main-realsense-rgbd.cpp)
  
  target_link_libraries(rs-single-example PRIVATE tek5030::rs2)
  target_link_libraries(rs-stereo-example PRIVATE tek5030::rs2)
  target_link_libraries(rs-rgbd-example   PRIVATE tek5030::rs2)
endif()
