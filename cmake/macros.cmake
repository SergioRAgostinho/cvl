# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/04
# \data last modified: 2017/05/04

# _name - the name to be applied to the target
# _srcs -  a list with the source files
# _deps - the list of lib dependencies, targets or libs
# 
macro (CVL_ADD_TEST _name _srcs _deps)
  set (TARGET_NAME "test_${_name}")
  set (RUNTIME_DIR "${CMAKE_BINARY_DIR}/tests/bin")

  #target creation
  add_executable(${TARGET_NAME} ${_srcs})
  target_link_libraries(${TARGET_NAME} ${GTEST_BOTH_LIBRARIES} ${_deps})
  set_target_properties(${TARGET_NAME}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${RUNTIME_DIR}")

  #test addition
  add_test (NAME ${TARGET_NAME}
            COMMAND "${TARGET_NAME}" "--gtest_color=yes"
            WORKING_DIRECTORY ${RUNTIME_DIR})
endmacro (CVL_ADD_TEST)