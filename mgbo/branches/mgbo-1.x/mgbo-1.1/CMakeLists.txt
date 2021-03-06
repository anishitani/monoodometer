cmake_minimum_required(VERSION 2.8)

project( mgbo )

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
find_package( OpenCV REQUIRED )
find_package( VISO2 REQUIRED )
find_package( CameraTool REQUIRED )
find_package( Boost COMPONENTS system filesystem REQUIRED )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set the tool src
set(TOOL src/tool)

set(CMAKE_CXX_FLAGS "-Wall -O3 -pg -msse3")

include_directories( include )

if(VISO2_FOUND)
    include_directories( ${VISO2_INCLUDE_DIR} )
endif(VISO2_FOUND)

if(CAMERA_TOOL_FOUND)
    include_directories( ${CAMERA_TOOL_INCLUDE_DIR} )
endif(CAMERA_TOOL_FOUND)

add_library( esm src/ESM.cpp include/math_tools.h )

# Tools for proofing
add_executable( write_xml ${TOOL}/write_xml.cpp )
add_executable( error_map ${TOOL}/error_map.cpp )

target_link_libraries( write_xml ${OpenCV_LIBS} )
target_link_libraries( error_map ${OpenCV_LIBS} )


# Odometry runners and loggers
#add_executable( KITTI src/KITTI.cpp )
#add_executable( alberta src/alberta.cpp )
#add_executable( malaga src/malaga.cpp )
add_executable( LRM src/LRM.cpp )
#add_executable( calibration src/vertical_pattern_calibration.cpp )
#add_executable( birdeyeview src/birdeyeview.cpp )
#add_executable( odom_validation src/odom_validation.cpp )

#target_link_libraries( KITTI esm ${OpenCV_LIBS}  ${VISO2_LIBS} ${CAMERA_TOOL_LIBS} )
#target_link_libraries( alberta esm ${OpenCV_LIBS}  ${VISO2_LIBS} ${CAMERA_TOOL_LIBS} )
#target_link_libraries( malaga esm ${OpenCV_LIBS}  ${VISO2_LIBS} ${CAMERA_TOOL_LIBS} )
target_link_libraries( LRM 
    esm 
    ${OpenCV_LIBS}  
    ${VISO2_LIBS} 
    #${CAMERA_TOOL_LIBS} 
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY} )
#target_link_libraries( calibration ${OpenCV_LIBS} ${CAMERA_TOOL_LIBS} )
#target_link_libraries( birdeyeview ${OpenCV_LIBS} ${CAMERA_TOOL_LIBS} ${VISO2_LIBS} )
#target_link_libraries( odom_validation ${OpenCV_LIBS} ${VISO2_LIBS} ${CAMERA_TOOL_LIBS} )
