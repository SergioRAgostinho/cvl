# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/03
# \data last modified: 2017/05/05

# Project configurations
cmake_minimum_required (VERSION 3.7)
project (cvl)


#compiler requirements
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

#warning settings 
if (CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
elseif (MSVC)
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
endif (CMAKE_COMPILER_IS_GNUCC)

# global setting
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
set (BUILD_SHARED_LIBS ON)

# Additional utilities
include ("cmake/macros.cmake")
include ("cmake/functions.cmake")

#check for dependencies
find_package (Threads REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (OpenCV 3.0 REQUIRED)
find_package (VTK REQUIRED)
find_package (TinyXML2)
find_package (Matlab COMPONENTS MAT_LIBRARY MX_LIBRARY)
find_package (GTest)

# Recurse into subdirectories
# Lib
add_subdirectory (common)
add_subdirectory (io)
# add_subdirectory (visualization)
# execs
add_subdirectory (examples)

#conditional addition of tests if gtest is found
if (GTEST_FOUND)
  enable_testing ()
  add_subdirectory (tests)
endif (GTEST_FOUND)