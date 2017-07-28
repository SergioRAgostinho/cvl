# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/07/27
# \data last modified: 2017/07/27


set (TARGET_NAME "${TARGET_NAME_PREFIX}_vgm_model")
cvl_dependency_check (BUILD_${TARGET_NAME}
												DEPENDS BUILD_${MODULE_NAME}
																BUILD_io
																Matlab_FOUND
												CUSTOM_MESSAGE "Not building ${TARGET_NAME}")

#target
if (BUILD_${TARGET_NAME})
	add_executable (${TARGET_NAME} "vgm_model.cpp")
	target_link_libraries (${TARGET_NAME}
														"${PROJECT_NAME}_io"
														${Matlab_LIBRARIES})
	target_include_directories (${TARGET_NAME}
																PRIVATE ${Matlab_INCLUDE_DIRS})
endif (BUILD_${TARGET_NAME})
