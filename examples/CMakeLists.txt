# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/05/05
# \data last modified: 2017/08/17

set (MODULE_NAME "examples")
set (TARGET_NAME_PREFIX "${PROJECT_NAME}_${MODULE_NAME}")

if (BUILD_${MODULE_NAME})
	add_subdirectory (mat_file_io)
	add_subdirectory (nanoflann)
	add_subdirectory (vgm_grabber)
	add_subdirectory (vgm_model)
endif (BUILD_${MODULE_NAME})
