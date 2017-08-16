# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/05
# \data last modified: 2017/07/26

#target
set (TARGET_NAME "${TARGET_NAME_PREFIX}_vgm_grabber")
cvl_dependency_check (BUILD_${TARGET_NAME}
												DEPENDS BUILD_io
																BUILD_filter
																BUILD_visualization
																Matlab_FOUND
												CUSTOM_MESSAGE "Not building ${TARGET_NAME}")

if (BUILD_${TARGET_NAME})
	add_executable (${TARGET_NAME} "vgm_grabber.cpp")
	target_link_libraries (${TARGET_NAME}
														"${PROJECT_NAME}_io"
														"${PROJECT_NAME}_filter"
														"${PROJECT_NAME}_visualization"
														${Matlab_LIBRARIES})
	target_include_directories (${TARGET_NAME} PRIVATE ${Matlab_INCLUDE_DIRS})
endif (BUILD_${TARGET_NAME})
