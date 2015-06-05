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

set(MYCXXFLAGS "CXXFLAGS=-g -O3 -fPIC")
set (CMAKE_C_FLAGS   "-O3" )
EXECUTE_PROCESS(COMMAND make ${MYCXXFLAGS} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty_libs/loki-0.1.7)

#common commands for building c++ executables and libraries
include_directories(3rdparty_libs/loki-0.1.7/include/loki)
find_library(mylibs loki 3rdparty_libs/loki-0.1.7/lib)
rosbuild_add_library(${PROJECT_NAME} src/binMap.cpp src/matFuns.cpp src/occupancyGridMap.cpp src/OGMros.cpp src/OGMReflectProb.cpp src/rangeSensorData.cpp src/ConfigFile.cpp)
set(MYCFLAGS "-fPIC")
rosbuild_add_compile_flags(${PROJECT_NAME} ${MYCFLAGS})
target_link_libraries(${PROJECT_NAME} ${mylibs})

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
