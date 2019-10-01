# setting the Gurobi properties for CMake modules
#
# FindGurobi will export an imported library interface target to link to
#
# GUROBI_DIR is the path to the Gurobi library directory
#
# Exported Target GUROBI

# define GUROBI_DIR as an "CACHE VARIABLE" and initialize
if(NOT GUROBI_DIR)
  if(DEFINED SDK_BASEPATH)
    set(_SDK_BASEPATH "${SDK_BASEPATH}")
  else()
    set(_SDK_BASEPATH "${CMAKE_INSTALL_PREFIX}")
  endif()
  set(GUROBI_DIR "${_SDK_BASEPATH}/Gurobi-8.0.0/win64/lib" CACHE PATH "Specifies the gurobi lib directory")
endif()

set(GUROBI_FOUND OFF)

if(NOT TARGET GUROBI AND GUROBI_DIR)
    # check, whether all files are there
	set(GUROBI_INCLUDE_DIR "${GUROBI_DIR}/../include/")
	set(GUROBI_LIBRARY_DIR "${GUROBI_DIR}")
	set(GUROBI_LIBRARIES "${GUROBI_LIBRARY_DIR}/gurobi_c++mt2015.lib;${GUROBI_LIBRARY_DIR}/gurobi80.lib")
	set(GUROBI_BIN_DIR "${GUROBI_DIR}/../bin/")
	   
	set(GUROBI_FOUND ON)
	foreach(lib ${GUROBI_LIBRARIES})
	  if(NOT EXISTS ${lib})
		message(STATUS "Gurobi library ${lib} missing")
		set(GUROBI_FOUND OFF)
      endif()
	endforeach()
		
	add_library(GUROBI INTERFACE IMPORTED)
	
	set_target_properties(GUROBI PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${GUROBI_INCLUDE_DIR}"
		)
else()
	set(GUROBI_DIR "")
endif()