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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#include_directories(3rdparty_libs/CGAL-3.9/include)
#include_directories(3rdparty_libs/mpfr-3.1.0/src)
#find_library(mylibs CGAL 3rdparty_libs/CGAL-3.9/lib)
#set(MYCFLAGS "-frounding-math")
#rosbuild_add_executable(partition src/partition.cpp)
#rosbuild_add_compile_flags(partition ${MYCFLAGS})
#target_link_libraries(partition ${mylibs})

set (CMAKE_C_FLAGS   "-O3" )
rosbuild_add_executable(active_people_search src/main.cpp src/active_people_search.cpp src/treeNode.cpp)

#target_link_libraries(example ${PROJECT_NAME})


