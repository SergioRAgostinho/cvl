# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/08/07
# \data last modified: 2017/08/17

#target
set (EXAMPLE_NAME "nanoflann")
set (TARGET_NAME "${TARGET_NAME_PREFIX}_${EXAMPLE_NAME}")
cmake_dependent_option (BUILD_${MODULE_NAME}_${EXAMPLE_NAME}
													"Enables the compilation of the ${EXAMPLE_NAME} example." ON
													"BUILD_${MODULE_NAME};BUILD_search;nanoflann_FOUND" OFF)


if (BUILD_${MODULE_NAME}_${EXAMPLE_NAME})
	add_executable (${TARGET_NAME} "nanoflann.cpp")
	target_link_libraries (${TARGET_NAME} ${PROJECT_NAME}_search)
	target_include_directories (${TARGET_NAME} PRIVATE	${nanoflann_INCLUDE_DIR}
																											${EIGEN3_INCLUDE_DIR})

	# install rules
	if (INSTALL_examples)
		install (TARGETS ${TARGET_NAME} RUNTIME DESTINATION "bin")
	endif (INSTALL_examples)
endif (BUILD_${MODULE_NAME}_${EXAMPLE_NAME})
