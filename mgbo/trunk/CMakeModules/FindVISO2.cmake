# - Try to find libCVD
#
#  VISO2_FOUND - system has libVISO2
#  VISO2_INCLUDE_DIR - the libVISO2 include directories
#  VISO2_LIBRARY - link these to use libVISO2

message(STATUS "Include path = ${CMAKE_INCLUDE_PATH}")

FIND_PATH(
  VISO2_INCLUDE_DIR
  NAMES viso_mono.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../libviso2/include
    ${CMAKE_SOURCE_DIR}/../viso2/include
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  VISO2_LIBS
  NAMES viso2
  PATHS
    ${CMAKE_LIBRARY_PATH}
    ${CMAKE_SOURCE_DIR}/../libviso2/lib
    ${CMAKE_SOURCE_DIR}/../viso2/lib
    /usr/lib
    /usr/local/lib
)

IF(VISO2_INCLUDE_DIR AND VISO2_LIBS)
  SET(VISO2_FOUND TRUE)
ENDIF()

MESSAGE(STATUS "Found VISO2: ${VISO2_INCLUDE_DIR}")

IF(VISO2_FOUND)
   IF(NOT VISO2_FIND_QUIETLY)
      MESSAGE(STATUS "Found VISO2: ${VISO2_LIBS}")
   ENDIF()
ELSE()
   IF(VISO2_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find VISO2")
   ENDIF()
ENDIF()