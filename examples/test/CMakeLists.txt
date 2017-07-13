# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/11
# \data last modified: 2017/05/11

#target
set (TARGET_NAME "${TARGET_NAME_PREFIX}_compilation_test")
add_executable (${TARGET_NAME} "compilation_test.cpp")
target_link_libraries (${TARGET_NAME} ${CMAKE_THREAD_LIBS_INIT})