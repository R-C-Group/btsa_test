cmake_minimum_required(VERSION 2.8.3)
project(btsa)

#SET(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -pthread")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

ADD_COMPILE_OPTIONS(-std=c++17 )
ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 14)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=12)
    message("core for MP: 14")
  elseif(N GREATER 10)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=8)
    message("core for MP: 8")
  elseif(N GREATER 7)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=6)
    message("core for MP: 6")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(Boost REQUIRED COMPONENTS timer)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# glog：无可靠 GlogConfig.cmake，且部分环境没有 FindGlog.cmake；用 pkg-config 与 Debian 包一致。
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)
find_package(gflags REQUIRED)
set(LOG_LIB ${GLOG_LIBRARIES} gflags)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
#find_package(GTSAM REQUIRED QUIET)

# VoxelHashMap 等使用 Intel TBB，仅包含头文件会在链接阶段报 undefined reference
find_library(TBB_LIBRARY NAMES tbb)
if(NOT TBB_LIBRARY)
  message(FATAL_ERROR "libtbb not found (Ubuntu: sudo apt install libtbb-dev)")
endif()

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

#add_message_files(
#  FILES
#  Pose6D.msg
#)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(btsa_node src/voxelMapping.cpp src/preprocess.cpp)
target_link_libraries(btsa_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${LOG_LIB} ${TBB_LIBRARY})
target_include_directories(btsa_node PRIVATE ${PYTHON_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})

add_executable(pandar_to_velodyne src/converter/pandar_to_velodyne.cpp)
target_link_libraries(pandar_to_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pandar_to_velodyne PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(rs_to_velodyne src/converter/rs_to_velodyne.cpp)
target_link_libraries(rs_to_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(rs_to_velodyne PRIVATE ${PYTHON_INCLUDE_DIRS})
