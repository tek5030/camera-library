cmake_minimum_required(VERSION 3.9.6)
project(tek5030 VERSION 2.0.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

include(cmake-boilerplate)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)

add_library(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
  PUBLIC ${OpenCV_LIBS}
  )
target_sources(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/dual_camera.h>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/kitti_camera.h>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/opencv_camera.h>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/stereo_pair.h>
  $<INSTALL_INTERFACE:include/tek5030/dual_camera.h>
  $<INSTALL_INTERFACE:include/tek5030/kitti_camera.h>
  $<INSTALL_INTERFACE:include/tek5030/opencv_camera.h>
  $<INSTALL_INTERFACE:include/tek5030/stereo_pair.h>
  PRIVATE
  src/dual_camera.cpp
  src/kitti_camera.cpp
  src/opencv_camera.cpp
  src/stereo_pair.cpp
  )

add_library_boilerplate()

option(WITH_REALSENSE "Build examples" ON)
if (NOT WITH_REALSENSE)
message(STATUS "* Component tek5030::rs2 with support for Intel® RealSense™ is DISABLED because option WITH_REALSENSE is ${WITH_REALSENSE}")
else()
  find_package(realsense2 QUIET)
  if (NOT realsense2_FOUND)
    message(WARNING "* Component tek5030::rs2 with support for Intel® RealSense™ is DISABLED because library realsense2 was not found")
  else()
    message(STATUS "* realsense2 was found: add_library(rs2)")
    add_library(rs2)
    target_link_libraries(rs2
      PUBLIC ${PROJECT_NAME}
      PRIVATE realsense2::realsense2
      )
    target_sources(rs2
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/realsense_single.h>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/realsense_rgbd.h>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/realsense_stereo.h>
      $<INSTALL_INTERFACE:include/tek5030/realsense_single.h>
      $<INSTALL_INTERFACE:include/tek5030/realsense_rgbd.h>
      $<INSTALL_INTERFACE:include/tek5030/realsense_stereo.h>
      PRIVATE
      src/realsense_camera_helpers.cpp
      src/realsense_single.cpp
      src/realsense_rgbd.cpp
      src/realsense_stereo.cpp
      )
    add_library_boilerplate(rs2)
  endif()
endif()

add_cmake_boilerplate()

option(BUILD_EXAMPLES "Build examples" OFF)
if (BUILD_EXAMPLES)
  add_subdirectory(example)
endif ()
