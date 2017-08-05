# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/07/30
# \data last modified: 2017/07/30

# module info
set (MODULE_NAME visualization)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

set (BUILD_${MODULE_NAME} TRUE
      CACHE BOOL
      "Enables the compilation of the ${MODULE_NAME} module.")

if (BUILD_${MODULE_NAME})
	#files
	set (headers
	        "include/cvl/visualization/opencv.h")
	# set (impls
	#         "include/cvl/common/impl/geometry.hpp")
	set (sources 	"src/opencv.cpp")

	#target creation
	add_library (${TARGET_NAME} ${sources})
	target_link_libraries (${TARGET_NAME} PUBLIC
																${PROJECT_NAME}_common
																${OpenCV_LIBRARIES})
	target_include_directories (${TARGET_NAME} PUBLIC
	                              "${CMAKE_CURRENT_SOURCE_DIR}/include"
	                              ${OpenCV_INCLUDE_DIRS})
	                              # ${EIGEN3_INCLUDE_DIR})
endif (BUILD_${MODULE_NAME})
