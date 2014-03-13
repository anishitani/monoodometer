# - Try to find my_camera_tools
#
#  CAMERA_TOOL_ - system has libCVD
#  CVD_INCLUDE_DIR - the libCVD include directories
#  CVD_LIBRARY - link these to use libCVD

FIND_PATH(
  CAMERA_TOOL_INCLUDE_DIR
  NAMES camera_tool.hpp
  PATHS
    ${CMAKE_SOURCE_DIR}/../my_camera_tool/include
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  CAMERA_TOOL_LIBS
  NAMES camera_tools
  PATHS
    ${CMAKE_SOURCE_DIR}/../my_camera_tool/lib
    /usr/lib
    /usr/local/lib
) 

IF(CAMERA_TOOL_INCLUDE_DIR AND CAMERA_TOOL_LIBS)
  SET(CAMERA_TOOL_FOUND TRUE)
ENDIF()

IF(CAMERA_TOOL_FOUND)
   IF(NOT CAMERA_TOOL_FIND_QUIETLY)
      MESSAGE(STATUS "Found CameraTool: ${CAMERA_TOOL_LIBS}")
   ENDIF()
ELSE()
   IF(CAMERA_TOOL_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find CameraTool")
   ENDIF()
ENDIF()