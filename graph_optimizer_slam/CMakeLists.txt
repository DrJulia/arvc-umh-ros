cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(eigen)
include_directories(${EIGEN_INCLUDE_DIRS})
link_libraries(${EIGEN_LIBRARIES})

# For building the CHOLMOD / CSPARSE solvers
FIND_PACKAGE(SuiteSparse)
IF(CSPARSE_FOUND)
  SET( BUILD_CSPARSE OFF CACHE BOOL "Build local CSparse library")
ELSE(CSPARSE_FOUND)
  SET( BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")
  SET(CSPARSE_FOUND TRUE)
ENDIF(CSPARSE_FOUND)
include_directories(${SuiteSparse_INCLUDE_DIRS})

link_libraries(-lg2o_core -lg2o_types_slam2d -lg2o_solver_csparse  -lcxsparse)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(${PROJECT_NAME} src/main.cpp src/graph_optimizer_slam.cpp src/global_graph.cpp)
#target_link_libraries(example ${PROJECT_NAME})
