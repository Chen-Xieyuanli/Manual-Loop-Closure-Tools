cmake_minimum_required(VERSION 3.0.2)
project(manual_loop_closure_backend)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(OpenMP QUIET)

find_path(
  GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/Geoid.hpp
  PATHS /usr/include /usr/local/include ${CMAKE_INSTALL_PREFIX}/include
)
if(NOT GEOGRAPHICLIB_INCLUDE_DIR)
  message(FATAL_ERROR "GeographicLib headers not found. Install libgeographic-dev.")
endif()

find_library(
  GEOGRAPHICLIB_LIBRARY Geographic
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu ${CMAKE_INSTALL_PREFIX}/lib ${CMAKE_INSTALL_PREFIX}/lib64
)
if(NOT GEOGRAPHICLIB_LIBRARY)
  message(FATAL_ERROR "libGeographic not found. Install libgeographic-dev.")
endif()

find_library(
  ZSTD_LIBRARY zstd
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu ${CMAKE_INSTALL_PREFIX}/lib ${CMAKE_INSTALL_PREFIX}/lib64
)
if(NOT ZSTD_LIBRARY)
  message(FATAL_ERROR "libzstd not found. Install libzstd-dev.")
endif()

find_package(
  catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    nav_msgs
    pcl_conversions
    roscpp
    rosbag
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf
    visualization_msgs
)
find_package(Boost REQUIRED COMPONENTS filesystem regex timer date_time program_options)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_transport
    nav_msgs
    pcl_conversions
    roscpp
    rosbag
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf
    visualization_msgs
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GEOGRAPHICLIB_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(
  manual_loop_optimize
  include/base_type.cpp
  src/data_saver.cpp
  src/common/runtime_logger.cpp
  src/ms_mapping/ms_manual_loop_optimize.cpp
)

target_link_libraries(
  manual_loop_optimize
  PRIVATE
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    gtsam
    ${GEOGRAPHICLIB_LIBRARY}
    ${Boost_LIBRARIES}
    ${ZSTD_LIBRARY}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(manual_loop_optimize PRIVATE OpenMP::OpenMP_CXX)
endif()

install(TARGETS manual_loop_optimize RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
