# \author Sergio Agostinho - sergio.r.agostinho@gmail.com
# \data created: 2017/07/25
# \data last modified: 2017/07/25

function (cvl_dependency_check _result)
	#parse all arguments
	set (options)
	set (oneValueArgs CUSTOM_MESSAGE)
	set (multiValueArgs DEPENDS)
	cmake_parse_arguments(VAR "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

	# Set true by default and abort if something fails
	set (${_result} TRUE PARENT_SCOPE)
	foreach(_var ${VAR_DEPENDS})
		if (NOT ${_var})
			set (${_result} FALSE PARENT_SCOPE)
			if (VAR_CUSTOM_MESSAGE)
				message (STATUS "${VAR_CUSTOM_MESSAGE} (reason: ${_var} - ${${_var}})")
			else (VAR_CUSTOM_MESSAGE)
				message (STATUS "Depency not met (reason: ${_var} - ${${_var}})")
			endif (VAR_CUSTOM_MESSAGE)
			break ()
		endif (NOT ${_var})
	endforeach(_var)
endfunction (cvl_dependency_check)
