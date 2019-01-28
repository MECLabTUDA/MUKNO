project(MukImaging)

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/)

set(Includes
	${INC_DIR}muk_imaging_api.h
	${INC_DIR}muk_imaging_tools.h
	${INC_DIR}MukImage.h
	${INC_DIR}MukImagingIO.h
	${INC_DIR}PluginManagerImaging.h
	${INC_DIR}VoxelCoordinate.h
	)
set(Private_Includes
	${INC_DIR}private/muk.pch
	)
set(Sources
	muk.cpp
	muk_imaging_tools.cpp
	MukImage.cpp
	MukImagingIO.cpp
	PluginManagerImaging.cpp
	VoxelCoordinate.cpp
	)
 
# configure the resource file
gris_configure_resource_rc(Resources MuknoII Imaging ${MODULE_NAME})

add_library(${PROJECT_NAME} SHARED 	
	${Private_Includes}
	${Includes}
	${Sources}
	${Resources}
	${Files}
	)

target_include_directories(${PROJECT_NAME} PUBLIC
	${INC_DIR}
	${EIGEN_INCLUDE_DIR}
	${GMP_INCLUDE_DIR}
	${CGAL_INCLUDE_DIR}
	${ITK_INCLUDE_DIR}
	)

include(${ITK_USE_FILE})
include(${VTK_USE_FILE})
	
target_link_libraries(${PROJECT_NAME}
	gstd
	MukCommon
	${ITK_LIBRARIES}
	${BOOST_TARGETS}
	)
	
source_group("Header Files\\private" FILES
	${Private_Includes}
	)

# ---------- deploy and install

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")

  
