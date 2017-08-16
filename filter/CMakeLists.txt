# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/08/11
# \data last modified: 2017/08/11

# module info
set (MODULE_NAME filter)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

set (BUILD_${MODULE_NAME} TRUE
      CACHE BOOL
      "Enables the compilation of the ${MODULE_NAME} module.")

if (BUILD_${MODULE_NAME})

  #Check dependencies
  cvl_dependency_check (_BUILD_${TARGET_NAME}
                          DEPENDS nanoflann_FOUND
                                  BUILD_search
                          CUSTOM_MESSAGE "Not building ${TARGET_NAME}")

  #files
  set (headers
          "include/cvl/filter/duplicate.h")
  set (implementations
          "include/cvl/filter/impl/duplicate.hpp")
  set (sources
          "src/duplicate.cpp")

  #target properie and creation
  add_library (${TARGET_NAME} ${sources})
  target_link_libraries (${TARGET_NAME} PUBLIC
                            ${PROJECT_NAME}_common
                            ${PROJECT_NAME}_search)
  target_include_directories (${TARGET_NAME} PUBLIC
                                "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (BUILD_${MODULE_NAME})
