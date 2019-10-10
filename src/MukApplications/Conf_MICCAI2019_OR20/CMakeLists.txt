# ------------------------------------------------------------------------------------------

project(Conf_MICCAI2019_OR20)

# ------------------------------------------------------------------------------------------

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${MODULE_NAME}/${PROJECT_NAME}/)

set(Includes
	${INC_DIR}MmwhsExperiment.h
	${INC_DIR}SegThorExperiment.h
	)	
	
set(Private_Includes
	${INC_DIR}private/CombinedStatistics.h
	${INC_DIR}private/custom_streams.h
	${INC_DIR}private/muk.pch
	${INC_DIR}private/program_functions.h
	${INC_DIR}private/program_options.h
	${INC_DIR}private/UnetResultRestrictionFilter.h
	)
	
set(Sources
	CombinedStatistics.cpp
	custom_streams.cpp
	main.cpp
	MmwhsExperiment.cpp
	SegThorExperiment.cpp
	#program_functions.cpp
	program_options.cpp
	UnetResultRestrictionFilter.cpp
	)

# ------------------------------------------------------------------------------------------

# QT stuff
set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON) # this is only necessary if qt designer is used
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5PrintSupport_DEFINITIONS})
	
include(${VTK_USE_FILE})
include(${ITK_USE_FILE})

# ------------------------------------------------------------------------------------------
# configure the resource file
gris_configure_resource_rc(Resources MuknoII Conf_MICCAI2019_OR20 ${MODULE_NAME})

add_executable(${PROJECT_NAME}
	${Private_Includes}
	${Includes}
	${Sources}
	)
	
target_include_directories(${PROJECT_NAME} PUBLIC
	${INC_DIR}
	${QCUSTOMPLOT_INCLUDE_DIR}
	)
	
target_link_libraries(${PROJECT_NAME} PUBLIC
	MukAppModels
	MukCommon
	MukConvexOptimization
	MukEvaluation
	MukImaging
	MukPathPlanning
	MukPdmWrapper
	MukQt
	MukVisualization
	MukOtobasisSegmentation
	gstd
	QCustomPlot
	${BOOST_TARGETS}
	${VTK_LIBRARIES}
	${Muk_Qt_Modules}
	)
		
if (${MukLibs_USE_INTERNAL_PDM})
else()
	include_directories(${PROJECT_NAME} PUBLIC
		${PDMLIB_INCLUDE_DIR}
		)
	
	target_link_libraries(${PROJECT_NAME} PUBLIC
		libremesh
		pdmlib
		)
endif()

source_group("Header Files\\private" FILES
	${Private_Includes}
	)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${MODULE_NAME})

# ---------- post build and install --------------------------------------------------------

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")
#if(NOT EXISTS "${MuknoII_Resources_DIR}/MukApplications/${PROJECT_NAME}/")
#	gris_deploy_directories(${PROJECT_NAME} "${MuknoII_Resources_DIR}/MukApplications/${PROJECT_NAME}/" "${CMAKE_SOURCE_DIR}/${MuknoII_Resources_DIR}/MukApplications/${PROJECT_NAME}/")
#endif()
gris_bundle(${PROJECT_NAME})