# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/05/03
# \data last modified: 2017/05/03

# Dep common to all common tests
set (deps "${PROJECT_NAME}_common")

cvl_add_test ("common_function" "function.cpp" ${deps})