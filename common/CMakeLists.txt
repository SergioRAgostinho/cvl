# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/05/03
# \data last modified: 2017/08/17

# module info
set (MODULE_NAME common)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

if (BUILD_${MODULE_NAME})
	#files
	set (headers
	        "include/cvl/common/algorithm.h"
	        "include/cvl/common/camera.h"
	        "include/cvl/common/common.h"
	        "include/cvl/common/eigen.h"
	        "include/cvl/common/function.h"
	        "include/cvl/common/geometry.h"
	        "include/cvl/common/mesh.h")
	set (impls
	        "include/cvl/common/impl/geometry.hpp")
	set (sources 	"src/function.cpp"
								"src/geometry.cpp"
								"src/mesh.cpp")

	#target creation
	add_library (${TARGET_NAME} ${sources})
	target_include_directories (${TARGET_NAME} PUBLIC
	                              "${CMAKE_CURRENT_SOURCE_DIR}/include"
	                              ${EIGEN3_INCLUDE_DIR})

	# install rules
	install (TARGETS ${TARGET_NAME}
						LIBRARY DESTINATION "lib"
						ARCHIVE DESTINATION "lib")
	install (FILES ${headers} DESTINATION "include/cvl/common")
	install (FILES ${impls} DESTINATION "include/cvl/common/impl")
endif (BUILD_${MODULE_NAME})
