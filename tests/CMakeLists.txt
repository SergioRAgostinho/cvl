# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/05/03
# \data last modified: 2017/07/25
set (MODULE_NAME tests)

set (BUILD_${MODULE_NAME} TRUE
      CACHE BOOL
      "Enables the compilation of the ${MODULE_NAME} module.")

if (BUILD_${MODULE_NAME})
	# global requirements for all tests
	include_directories(${GTEST_INCLUDE_DIRS})

	#include the individual modules
	add_subdirectory (common)
	add_subdirectory (io)
endif (BUILD_${MODULE_NAME})
