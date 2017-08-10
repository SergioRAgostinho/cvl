# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/08/07
# \data last modified: 2017/08/07
#
# Nano find script
#
# Sets the following variables:
#
# nanoflann_FOUND - set to true if the library is found
# nanoflann_INCLUDE_DIR - include file directory
#

# piggyback on pkgconfif for now
if ((NOT nanoflann_FOUND) AND PKG_CONFIG_FOUND)
  pkg_check_modules(nanoflann nanoflann)
endif ((NOT nanoflann_FOUND) AND PKG_CONFIG_FOUND)

if (NOT nanoflann_FOUND)
	# search for the folder
	find_path (nanoflann_INCLUDE_DIR nanoflann.hpp
								HINTS "${nanoflann_ROOT}" "${nanoflann_DIR}" "${nanoflann_PREFIX}"
								PATH_SUFFIXES include)

  find_package_handle_standard_args (nanoflann DEFAULT_MSG nanoflann_INCLUDE_DIR)
endif (NOT nanoflann_FOUND)
