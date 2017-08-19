# \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
# \data created: 2017/05/05
# \data last modified: 2017/05/05
#
# TinyXML2 find script
# 
# Sets the following variables:
#
# TinyXML2_FOUND - set to true if the library is found
# TinyXML2_INCLUDE_DIRS - include file directory
# TinyXML2_LIBRARIES - full path to the library
#

# piggyback on pkgconfif for now
if (PKG_CONFIG_FOUND)
  pkg_check_modules(TinyXML2 tinyxml2)
endif (PKG_CONFIG_FOUND)

if (NOT TinyXML2_FOUND)
  find_package_handle_standard_args (TinyXML2 FOUND_VAR TinyXML2_FOUND
                                            	REQUIRED_VARS "TinyXML2_INCLUDE_DIRS"
                                              	            "TinyXML2_LIBRARIES")
endif (NOT TinyXML2_FOUND)
