# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/03
# \data last modified: 2017/05/09

# module info
set (MODULE_NAME common)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

set (BUILD_${MODULE_NAME} TRUE
      CACHE BOOL
      "Enables the compilation of the ${MODULE_NAME} module.")

if (BUILD_${MODULE_NAME})
	#files
	set (headers
	        "include/cvl/common/algorithm.h"
	        "include/cvl/common/camera.h"
	        "include/cvl/common/eigen.h"
	        "include/cvl/common/function.h"
	        "include/cvl/common/geometry.h")
	set (impls
	        "include/cvl/common/impl/geometry.hpp")
	set (sources 	"src/function.cpp"
								"src/mesh.cpp")

	#target creation
	add_library (${TARGET_NAME} ${sources})
	target_include_directories (${TARGET_NAME} PUBLIC
	                              "${CMAKE_CURRENT_SOURCE_DIR}/include"
	                              ${EIGEN3_INCLUDE_DIR})
endif (BUILD_${MODULE_NAME})
