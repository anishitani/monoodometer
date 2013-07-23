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
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# creates a new library using the odometer, montion, frame and feature sources 
rosbuild_add_library( odometry_tools src/mono_odometer.cpp src/parameter.cpp src/image_processor.cpp src/motion_processor.cpp )

# creates the executables
#rosbuild_add_executable(feature_test src/feature_test.cpp)
rosbuild_add_executable(motion_test src/motion_test.cpp)
rosbuild_add_executable(motion_test2 src/motion_test2.cpp)
rosbuild_add_executable(motion_test3 src/motion_test3.cpp)
rosbuild_add_executable(motion_test4 src/motion_test4.cpp)
rosbuild_add_executable(motion_test5 src/motion_test5.cpp)
rosbuild_add_executable(${PROJECT_NAME} src/main.cpp)

#link boost
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

# links the odometer library to the project 
#target_link_libraries( feature_test odometry_tools )
target_link_libraries( motion_test odometry_tools )
target_link_libraries( motion_test2 odometry_tools )
target_link_libraries( motion_test3 odometry_tools )
target_link_libraries( motion_test4 odometry_tools )
target_link_libraries( motion_test5 odometry_tools )
target_link_libraries( ${PROJECT_NAME} odometry_tools )
