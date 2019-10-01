# ------------------------------------------------------------------------------------------

project(Conf_MICCAI2019)

# ------------------------------------------------------------------------------------------

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${MODULE_NAME}/${PROJECT_NAME}/)

set(Includes
	${INC_DIR}PlanningController.h
	${INC_DIR}program_options.h
	${INC_DIR}SegThorExperiment.h
	${INC_DIR}SharedPlanningExperiment.h
	)
	
set(Private_Includes
	${INC_DIR}private/muk.pch
	)
	
set(Sources
	main.cpp
	PlanningController.cpp
	program_options.cpp
	SegThorExperiment.cpp
	SharedPlanningExperiment.cpp
	)

# ------------------------------------------------------------------------------------------
	
# configure the resource file
gris_configure_resource_rc(Resources MuknoII Conf_MICCAI2019 ${MODULE_NAME})
	
include(${VTK_USE_FILE})
include(${ITK_USE_FILE})

# ------------------------------------------------------------------------------------------

add_executable(${PROJECT_NAME}
	${Private_Includes}
	${Includes}
	${Sources}
	)
	
target_include_directories(${PROJECT_NAME} PUBLIC
	${INC_DIR}
	)
	
target_link_libraries(${PROJECT_NAME}
	MukAppModels
	MukCommon
	MukConvexOptimization
	MukEvaluation
	MukImaging
	MukPathPlanning
	MukVisualization
	gstd
	${BOOST_TARGETS}
	${VTK_LIBRARIES}
	)

# ------------------------------------------------------------------------------------------

#source_group("Header Files\\private" FILES	${Private_Includes}	)
source_group("Header Files\\private" FILES	${Private_Includes}	)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${MODULE_NAME})

# -----------post build and install --------------------------------------------------------

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")
gris_deploy_directories(${PROJECT_NAME} "${MuknoII_Resources_DIR}/MukApplications/${PROJECT_NAME}/" "${CMAKE_SOURCE_DIR}/${MuknoII_Resources_DIR}/MukApplications/${PROJECT_NAME}/")

set(Bin_Lookup_Directories 
	${QCUSTOMPLOT_BIN_DIR}	
  ${Boost_LIBRARY_DIRS}
  ${GMP_BIN_DIR}
	)

gris_bundle_add_lookup_directories(${Bin_Lookup_Directories})

gris_bundle(${PROJECT_NAME})