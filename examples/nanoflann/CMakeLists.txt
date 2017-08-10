# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/08/07
# \data last modified: 2017/08/07

#target
set (TARGET_NAME "${TARGET_NAME_PREFIX}_nanoflann")
cvl_dependency_check (BUILD_${TARGET_NAME}
												DEPENDS nanoflann_FOUND
																BUILD_search
												CUSTOM_MESSAGE "Not building ${TARGET_NAME}")

if (BUILD_${TARGET_NAME})
	add_executable (${TARGET_NAME} "nanoflann.cpp")
	target_link_libraries (${TARGET_NAME} ${PROJECT_NAME}_search)
	target_include_directories (${TARGET_NAME} PRIVATE	${nanoflann_INCLUDE_DIR}
																											${EIGEN3_INCLUDE_DIR})
endif (BUILD_${TARGET_NAME})
