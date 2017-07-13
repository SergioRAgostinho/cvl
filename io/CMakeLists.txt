# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/05/03
# \data last modified: 2017/05/05

# module info
set (MODULE_NAME io)
set (TARGET_NAME "${PROJECT_NAME}_${MODULE_NAME}")

#files
set (headers
        "include/cvl/io/grabber.h")
set (sources
        "src/grabber.cpp")

if (TinyXML2_FOUND)
  list (APPEND headers "include/cvl/io/vgm_grabber.h")
  list (APPEND sources "src/vgm_grabber.cpp")
endif (TinyXML2_FOUND)

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
                          PUBLIC ${TinyXML2_LIBRARIES})
  target_include_directories (${TARGET_NAME}
                                PUBLIC ${TinyXML2_INCLUDE_DIRS})
endif (TinyXML2_FOUND)


