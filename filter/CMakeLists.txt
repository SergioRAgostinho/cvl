# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/08/11
# \data last modified: 2017/08/17

# module info
set (MODULE_NAME filter)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

if (BUILD_${MODULE_NAME})

  #files
  set (headers
          "include/cvl/filter/duplicate.h"
          "include/cvl/filter/filter.h")
  set (impls
          "include/cvl/filter/impl/duplicate.hpp")

  #target properie and creation
  add_library (${TARGET_NAME} INTERFACE)
  target_link_libraries (${TARGET_NAME} INTERFACE
                            ${PROJECT_NAME}_common
                            ${PROJECT_NAME}_search)
  target_include_directories (${TARGET_NAME} INTERFACE
                                "${CMAKE_CURRENT_SOURCE_DIR}/include")

  # install rules
  install (FILES ${headers} DESTINATION "include/cvl/filter")
  install (FILES ${impls} DESTINATION "include/cvl/filter/impl")
endif (BUILD_${MODULE_NAME})
