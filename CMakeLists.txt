# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/05/03
# \data last modified: 2017/08/18

# Project configurations
cmake_minimum_required (VERSION 3.7)
project (cvl VERSION 0.1)

#compiler requirements
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

#detect compiler
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(COMPILER_IS_CLANG TRUE)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
  set(COMPILER_IS_APPLE_CLANG TRUE)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(COMPILER_IS_GCC TRUE)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  set(COMPILER_IS_INTEL TRUE)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  set(COMPILER_IS_MSCV TRUE)
endif()

#warning settings
if (COMPILER_IS_GCC OR COMPILER_IS_CLANG OR COMPILER_IS_APPLE_CLANG)
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
elseif (COMPILER_IS_MSCV)
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
endif (COMPILER_IS_GCC OR COMPILER_IS_CLANG OR COMPILER_IS_APPLE_CLANG)

# global setting
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# Additional utilities
include (CMakeDependentOption)
include ("cmake/macros.cmake")
include ("cmake/functions.cmake")

#check for dependencies
find_package (PkgConfig)
find_package (Threads REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (OpenCV 3.0 REQUIRED core videoio highgui imgproc)
find_package (nanoflann)
find_package (TinyXML2)
find_package (Matlab COMPONENTS MAT_LIBRARY MX_LIBRARY)
find_package (GTest)
find_package (Doxygen 1.8.0)

# Options
option (BUILD_SHARED_LIBS "Selects if the library is build as shared library or a static one." ON)

option (BUILD_common "Enables the compilation of the common module." ON)
option (BUILD_io "Enables the compilation of the io module." ON)
option (BUILD_visualization "Enables the compilation of the visualization module." ON)
cmake_dependent_option (BUILD_search
                          "Enables the compilation of the search module." ON
                          "nanoflann_FOUND" OFF)
cmake_dependent_option (BUILD_filter
                          "Enables the compilation of the filter module." ON
                          "BUILD_search" OFF)

option (BUILD_examples "Enables the compilation of the examples." ON)
option (INSTALL_examples "Selects if the examples are to be installed." OFF)

# Recurse into subdirectories
# Lib
add_subdirectory (common)
add_subdirectory (io)
add_subdirectory (filter)
add_subdirectory (search)
add_subdirectory (visualization)
# execs
if (BUILD_examples)
	add_subdirectory (examples)
endif (BUILD_examples)

#conditional addition of tests if gtest is found
if (GTEST_FOUND)
  enable_testing ()
  add_subdirectory (tests)
endif (GTEST_FOUND)

# uninstall target
configure_file(
    "cmake/uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

# docs
if (Doxygen_FOUND)
  configure_file(
      "doc/Doxyfile.in"
      "${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile"
      IMMEDIATE @ONLY)
  add_custom_target (doc
      COMMAND ${DOXYGEN_EXECUTABLE}
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/doc"
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM)
endif (Doxygen_FOUND)
