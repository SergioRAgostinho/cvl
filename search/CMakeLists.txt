# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/08/07
# \data last modified: 2017/08/17

set (MODULE_NAME search)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

if (BUILD_${MODULE_NAME})

	#files
	set (headers "include/cvl/search/kdtree.h")
	set (impls "include/cvl/search/impl/kdtree.hpp")

	#target properties and creations
	add_library (${TARGET_NAME} INTERFACE)
	target_link_libraries (${TARGET_NAME} INTERFACE
																"${PROJECT_NAME}_common")
	target_include_directories (${TARGET_NAME} INTERFACE
                                "${CMAKE_CURRENT_SOURCE_DIR}/include"
                                ${nanoflann_INCLUDE_DIR})

	# install rules
	install (FILES ${headers} DESTINATION "include/cvl/search")
	install (FILES ${impls} DESTINATION "include/cvl/search/impl")
endif (BUILD_${MODULE_NAME})
