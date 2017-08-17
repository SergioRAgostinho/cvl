# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/03
# \data last modified: 2017/08/17

# module info
set (MODULE_NAME io)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

if (BUILD_${MODULE_NAME})
  #files
  set (headers
          "include/cvl/io/grabber.h"
          "include/cvl/io/utils.h")
  set (impls
          "include/cvl/io/impl/grabber.hpp")
  set (sources
          "src/grabber.cpp"
          "src/utils.cpp")

  if (Matlab_FOUND)
    list (APPEND headers  "include/cvl/io/vgm_model_reader.h")
    list (APPEND sources  "src/vgm_model_reader.cpp")

    if (TinyXML2_FOUND)
      list (APPEND headers "include/cvl/io/vgm_grabber.h")
      list (APPEND sources "src/vgm_grabber.cpp")
    endif (TinyXML2_FOUND)
  endif (Matlab_FOUND)

  #target properie and creation
  add_library (${TARGET_NAME} ${sources})
  target_link_libraries (${TARGET_NAME} PUBLIC
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

  # install rules
  install (TARGETS ${TARGET_NAME}
            LIBRARY DESTINATION "lib"
            ARCHIVE DESTINATION "lib")
  install (FILES ${headers} DESTINATION "include/cvl/io")
  install (FILES ${impls} DESTINATION "include/cvl/io/impl")
endif (BUILD_${MODULE_NAME})
