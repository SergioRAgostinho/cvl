# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/03
# \data last modified: 2017/05/03

# global requirements for all tests
include_directories(${GTEST_INCLUDE_DIRS})
set (MODULE_NAME tests)

#include the individual modules
add_subdirectory (common)
add_subdirectory (io)