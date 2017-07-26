# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/05
# \data last modified: 2017/07/25

set (MODULE_NAME "examples")
set (TARGET_NAME_PREFIX "${PROJECT_NAME}_${MODULE_NAME}")

set (BUILD_${MODULE_NAME} TRUE
      CACHE BOOL
      "Enables the compilation of the ${MODULE_NAME} module.")

if (BUILD_${MODULE_NAME})
	add_subdirectory (mat_file_io)
	# add_subdirectory (vgm_grabber)
	add_subdirectory (test)
endif (BUILD_${MODULE_NAME})
