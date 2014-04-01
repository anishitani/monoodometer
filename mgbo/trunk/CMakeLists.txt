cmake_minimum_required(VERSION 2.8)

project( mgbo )

include( ExternalProject )

set(CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT ON)

# Compilation flags
# msse3 needed for libviso2
set(CMAKE_CXX_FLAGS "-Wall -O3 -pg -msse3")

#############################
# Finding system libraries
############################# 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
find_package( OpenCV REQUIRED )
find_package( CameraTool REQUIRED )
find_package( Boost COMPONENTS system filesystem REQUIRED )

#######################
# External libraries
#######################
set( MGBO_EXTERNAL_PACKAGES_DIR "${PROJECT_SOURCE_DIR}/ext" )

# LIBVISO2
#add_subdirectory( ${LIBVISO2} )
ExternalProject_Add (
       libviso2
       SOURCE_DIR ${MGBO_EXTERNAL_PACKAGES_DIR}/libviso2
       BUILD_IN_SOURCE true
       INSTALL_COMMAND "" )
ExternalProject_Get_Property(libviso2 SOURCE_DIR)
include_directories( ${SOURCE_DIR}/include )
link_directories( ${SOURCE_DIR}/lib )

# Relative Pose Estimation
# Biblioteca de métodos de estimação da pose relativa
# baseado em SfM
ExternalProject_Add (
       relative-pose-estimation
       GIT_REPOSITORY https://github.com/prclibo/relative-pose-estimation.git
       GIT_TAG master
       SOURCE_DIR ${MGBO_EXTERNAL_PACKAGES_DIR}/relative-pose-estimation
       BUILD_IN_SOURCE true 
       INSTALL_COMMAND "" )
ExternalProject_Get_Property(relative-pose-estimation SOURCE_DIR)
include_directories(${SOURCE_DIR}/include)

# Set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Folder with the source code for a set of tools for
# used for testing, debugging, profiling and other
# funcionalities.
set(TOOLS src/tool)

# Directory with the project headers
include_directories( include )

# Include the camera tools headers if needed
if(CAMERA_TOOL_FOUND)
    include_directories( ${CAMERA_TOOL_INCLUDE_DIR} )
endif(CAMERA_TOOL_FOUND)

# Build a library for some operations like Matlab expm,
# matrix skew operation and others 
add_library( math_tools ${TOOLS}/math_tools.cpp  )
set( LIBS ${LIBS} math_tools )

# Library for the ESM methods
add_library( esm src/ESM.cpp )
target_link_libraries( esm math_tools )
set( LIBS ${LIBS} esm )

# Something about writing a XML file with OpenCV
#add_executable( write_xml ${TOOLS}/write_xml.cpp )
#set( BINS ${BINS} write_xml )
#target_link_libraries( write_xml ${OpenCV_LIBS} )

add_executable( LRM src/LRM.cpp )
target_link_libraries( LRM 
    esm
    viso2
    ${OpenCV_LIBS}   
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY} )
add_dependencies( LRM libviso2 )

# Odometry runners and loggers
#add_executable( KITTI src/KITTI.cpp )
#add_executable( alberta src/alberta.cpp )
#add_executable( malaga src/malaga.cpp )

#add_executable( calibration src/vertical_pattern_calibration.cpp )
#add_executable( birdeyeview src/birdeyeview.cpp )
#add_executable( odom_validation src/odom_validation.cpp )

#target_link_libraries( KITTI esm ${OpenCV_LIBS}  ${VISO2_LIBS} ${CAMERA_TOOL_LIBS} )
#target_link_libraries( alberta esm ${OpenCV_LIBS}  ${VISO2_LIBS} ${CAMERA_TOOL_LIBS} )
#target_link_libraries( malaga esm ${OpenCV_LIBS}  ${VISO2_LIBS} ${CAMERA_TOOL_LIBS} )
#target_link_libraries( calibration ${OpenCV_LIBS} ${CAMERA_TOOL_LIBS} )
#target_link_libraries( birdeyeview ${OpenCV_LIBS} ${CAMERA_TOOL_LIBS} ${VISO2_LIBS} )
#target_link_libraries( odom_validation ${OpenCV_LIBS} ${VISO2_LIBS} ${CAMERA_TOOL_LIBS} )



####################
# Installing libs
####################

# No installation rule defined
install( TARGETS 
    RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/bin 
    LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/lib 
    ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/lib )