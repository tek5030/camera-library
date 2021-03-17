cmake_minimum_required(VERSION 3.5.1)
project(tek5030 VERSION 1.1.0)
message(STATUS "* Generating '${PROJECT_NAME}' v${${PROJECT_NAME}_VERSION}")

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)

add_library(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/dual_camera.h>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/opencv_camera.h>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/stereo_pair.h>
  $<INSTALL_INTERFACE:include/tek5030/dual_camera.h>
  $<INSTALL_INTERFACE:include/tek5030/opencv_camera.h>
  $<INSTALL_INTERFACE:include/tek5030/stereo_pair.h>
  PRIVATE
  src/dual_camera.cpp
  src/opencv_camera.cpp
  src/stereo_pair.cpp
  )

set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_LINKER_LANGUAGE CXX
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 17
  )

target_compile_features(${PROJECT_NAME}
  PUBLIC cxx_std_17
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

target_link_libraries(${PROJECT_NAME}
  PUBLIC ${OpenCV_LIBS}
  )

find_package(realsense2 QUIET)
if (realsense2_FOUND)
  target_sources(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/realsense_single.h>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/realsense_rgbd.h>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tek5030/realsense_stereo.h>
    $<INSTALL_INTERFACE:include/tek5030/realsense_single.h>
    $<INSTALL_INTERFACE:include/tek5030/realsense_rgbd.h>
    $<INSTALL_INTERFACE:include/tek5030/realsense_stereo.h>
    PRIVATE
    src/realsense_camera_helpers.h
    src/realsense_single.cpp
    src/realsense_rgbd.cpp
    src/realsense_stereo.cpp
    )
  target_link_libraries(${PROJECT_NAME}
    PRIVATE realsense2::realsense2
    )
endif()

###

string(TOLOWER ${CMAKE_PROJECT_NAME} package_name)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_DEBUG_POSTFIX "d")
set(config_install_dir  "share/cmake/${CMAKE_PROJECT_NAME}/")
set(targets_export_name "${package_name}-targets")
set(project_config      "${package_name}-config.cmake")
set(version_config      "${package_name}-config-version.cmake")
set(namespace           "tek5030")

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake.in
  ${CMAKE_BINARY_DIR}/${project_config}
  INSTALL_DESTINATION ${config_install_dir}
  NO_SET_AND_CHECK_MACRO
)

write_basic_package_version_file(
  ${CMAKE_BINARY_DIR}/${version_config}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_BINARY_DIR}/${project_config}
  ${CMAKE_BINARY_DIR}/${version_config}
  DESTINATION ${config_install_dir}
  )

install(
  DIRECTORY "include/"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h"
  )

install(TARGETS ${PROJECT_NAME}
  EXPORT ${targets_export_name}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

install(EXPORT ${targets_export_name}
  NAMESPACE "${namespace}::"
  DESTINATION ${config_install_dir}
  )

export(
  TARGETS ${PROJECT_NAME}
  NAMESPACE "${namespace}::"
  FILE ${CMAKE_BINARY_DIR}/${targets_export_name}.cmake
)
