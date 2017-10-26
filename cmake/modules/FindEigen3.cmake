# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/10/26
# \data last modified: 2017/10/26
# \comment Slightly modified from PCL's repo

###############################################################################
# Find Eigen3
#
# This sets the following variables:
# EIGEN3_FOUND - True if Eigen was found.
# EIGEN3_INCLUDE_DIRS - Directories containing the Eigen include files.
# EIGEN3_DEFINITIONS - Compiler flags for Eigen.
# EIGEN3_VERSION - Package version

find_package(PkgConfig QUIET)
pkg_check_modules(PC_EIGEN eigen3)
set(EIGEN3_DEFINITIONS ${PC_EIGEN_CFLAGS_OTHER})

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} /usr /usr/local)
endif(CMAKE_SYSTEM_NAME STREQUAL Linux)
if(APPLE)
  list(APPEND CMAKE_INCLUDE_PATH /opt/local)
  set(CMAKE_FIND_FRAMEWORK NEVER)
endif()

find_path(EIGEN3_INCLUDE_DIR Eigen/Core
    HINTS ${PC_EIGEN_INCLUDEDIR} ${PC_EIGEN_INCLUDE_DIRS} "${EIGEN_ROOT}" "$ENV{EIGEN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/Eigen" "$ENV{PROGRAMW6432}/Eigen"
          "$ENV{PROGRAMFILES}/Eigen3" "$ENV{PROGRAMW6432}/Eigen3"
    PATH_SUFFIXES eigen3 include/eigen3 include)

if(EIGEN3_INCLUDE_DIR)
  file(READ "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" _eigen_version_header)

  string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen_world_version_match "${_eigen_version_header}")
  set(EIGEN3_WORLD_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen_major_version_match "${_eigen_version_header}")
  set(EIGEN3_MAJOR_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen_minor_version_match "${_eigen_version_header}")
  set(EIGEN3_MINOR_VERSION "${CMAKE_MATCH_1}")
  set(EIGEN3_VERSION ${EIGEN3_WORLD_VERSION}.${EIGEN3_MAJOR_VERSION}.${EIGEN3_MINOR_VERSION})
endif(EIGEN3_INCLUDE_DIR)

set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(CMAKE_FIND_FRAMEWORK)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3 DEFAULT_MSG EIGEN3_INCLUDE_DIR)

mark_as_advanced(EIGEN3_INCLUDE_DIR)

if(EIGEN3_FOUND)
  message(STATUS "Eigen3 found (include: ${EIGEN3_INCLUDE_DIRS}, version: ${EIGEN3_VERSION})")
endif(EIGEN3_FOUND)
