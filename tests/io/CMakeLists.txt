# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/05/04
# \data last modified: 2017/05/04

# Dep common to all common tests
set (deps "${PROJECT_NAME}_io")

cvl_add_test ("io_grabber" "grabber.cpp" ${deps})