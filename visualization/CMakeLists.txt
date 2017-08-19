# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/07/30
# \data last modified: 2017/08/17

# module info
set (MODULE_NAME visualization)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

if (BUILD_${MODULE_NAME})

	#files
	set (headers
	        "include/cvl/visualization/opencv.h")
	set (sources 	"src/opencv.cpp")

	#target creation
	add_library (${TARGET_NAME} ${sources})
	target_link_libraries (${TARGET_NAME} PUBLIC
																${PROJECT_NAME}_common
																${OpenCV_LIBRARIES})
	target_include_directories (${TARGET_NAME} PUBLIC
	                              "${CMAKE_CURRENT_SOURCE_DIR}/include"
	                              ${OpenCV_INCLUDE_DIRS})

  # install rules
	install (TARGETS ${TARGET_NAME}
						LIBRARY DESTINATION "lib"
						ARCHIVE DESTINATION "lib")
	install (FILES ${headers} DESTINATION "include/cvl/visualization")
endif (BUILD_${MODULE_NAME})
