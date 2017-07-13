# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/05
# \data last modified: 2017/05/05

#target
set (TARGET_NAME "${TARGET_NAME_PREFIX}_vgm_grabber")
add_executable (${TARGET_NAME} "vgm_grabber.cpp")
target_link_libraries (${TARGET_NAME} "${PROJECT_NAME}_io")