project(Conf_IPCAI2019)

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${MODULE_NAME}/${PROJECT_NAME}/)

set(Includes
	${INC_DIR}CochleaExperiment.h
	${INC_DIR}Experiment.h
	${INC_DIR}SegmentationEvaluator.h
	${INC_DIR}VestibularExperiment.h
	)	
	
set(Private_Includes
	${INC_DIR}private/muk.pch
	${INC_DIR}private/program_functions.h
	${INC_DIR}private/program_options.h
	)
	
set(Sources
	CochleaExperiment.cpp
	Experiment.cpp
	main.cpp
	program_functions.cpp
	program_options.cpp
	SegmentationEvaluator.cpp
	VestibularExperiment.cpp
	)

# QT stuff
set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON) # this is only necessary if qt designer is used
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5PrintSupport_DEFINITIONS})

add_executable(${PROJECT_NAME}
	${Private_Includes}
	${Includes}
	${Sources}
	)
	
include(${VTK_USE_FILE})
include(${ITK_USE_FILE})
	
target_include_directories(${PROJECT_NAME} PUBLIC
	${INC_DIR}
	)
	
target_link_libraries(${PROJECT_NAME}
	MukAppModels
	MukCommon
	MukEvaluation
	MukImaging
	MukPathPlanning
	MukQt
	MukVisualization
	gstd
	${BOOST_TARGETS}
	${VTK_LIBRARIES}
	${Muk_Qt_Modules}
	)

source_group("Header Files\\private" FILES
	${Private_Includes}
	)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${MODULE_NAME})

# ---------- post build and install
gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")
#gris_deploy_directories(${PROJECT_NAME} "${MuknoII_Resources_DIR}/MukApplications/${PROJECT_NAME}/" "${CMAKE_SOURCE_DIR}/${MuknoII_Resources_DIR}/MukApplications/${PROJECT_NAME}/")
gris_bundle(${PROJECT_NAME})