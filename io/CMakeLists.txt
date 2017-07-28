# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/03
# \data last modified: 2017/05/05

# module info
set (MODULE_NAME io)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

set (BUILD_${MODULE_NAME} TRUE
      CACHE BOOL
      "Enables the compilation of the ${MODULE_NAME} module.")

if (BUILD_${MODULE_NAME})
  #files
  set (headers
          "include/cvl/io/grabber.h"
          "include/cvl/io/utils.h")
  set (sources
          "src/grabber.cpp"
          "src/utils.cpp")

  if (Matlab_FOUND)
    list (APPEND headers  "include/cvl/io/mat_io.h"
                          "include/cvl/io/vgm_model_reader.h")
    list (APPEND sources  "src/mat_io.cpp"
                          "src/vgm_model_reader.cpp")

    if (TinyXML2_FOUND)
      list (APPEND headers "include/cvl/io/vgm_grabber.h")
      list (APPEND sources "src/vgm_grabber.cpp")
    endif (TinyXML2_FOUND)
  endif (Matlab_FOUND)

  #target properie and creation
  add_library (${TARGET_NAME} ${sources})
  target_link_libraries (${TARGET_NAME}
                            PUBLIC
                              ${PROJECT_NAME}_common
                              ${OpenCV_LIBRARIES})
  target_include_directories (${TARGET_NAME} PUBLIC
                                "${CMAKE_CURRENT_SOURCE_DIR}/include"
                                ${OpenCV_INCLUDE_DIR})

  if (TinyXML2_FOUND)
    target_link_libraries (${TARGET_NAME}
                              PRIVATE ${TinyXML2_LDFLAGS})
    target_include_directories (${TARGET_NAME}
                                  PRIVATE ${TinyXML2_INCLUDE_DIRS})
  endif (TinyXML2_FOUND)

  if (Matlab_FOUND)
    target_link_libraries (${TARGET_NAME}
                              PRIVATE ${Matlab_LIBRARIES})
    target_include_directories (${TARGET_NAME}
                                  PRIVATE ${Matlab_INCLUDE_DIRS})
  endif (Matlab_FOUND)
endif (BUILD_${MODULE_NAME})
