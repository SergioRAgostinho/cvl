# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/08/07
# \data last modified: 2017/08/07

set (MODULE_NAME search)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

set (BUILD_${MODULE_NAME} TRUE
      CACHE BOOL
      "Enables the compilation of the ${MODULE_NAME} module.")

if (BUILD_${MODULE_NAME})

	#Check dependencies
	cvl_dependency_check (_BUILD_${TARGET_NAME}
													DEPENDS nanoflann_FOUND
													CUSTOM_MESSAGE "Not building ${TARGET_NAME}")
	if (NOT _BUILD_${TARGET_NAME})
		set (BUILD_${MODULE_NAME} FALSE
		      CACHE BOOL
		      "Enables the compilation of the ${MODULE_NAME} module."
		      FORCE)
		return ()
	endif (NOT _BUILD_${TARGET_NAME})

	#files
	set (incls "include/cvl/search/kdtree.h")
	set (impls "include/cvl/search/impl/kdtree.hpp")
	set (srcs "src/kdtree.cpp")

	#target properties and creations
	add_library (${TARGET_NAME} INTERFACE)
	target_link_libraries (${TARGET_NAME} INTERFACE
																"${PROJECT_NAME}_common")
	target_include_directories (${TARGET_NAME} INTERFACE
                                "${CMAKE_CURRENT_SOURCE_DIR}/include"
                                ${nanoflann_INCLUDE_DIR})
endif (BUILD_${MODULE_NAME})