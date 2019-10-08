project(MukAppModels)
	
set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${MODULE_NAME}/)

set(Includes
	${INC_DIR}muk_appmodels_api.h
	${INC_DIR}AlgorithmModel.h
	${INC_DIR}ApplicationModel.h
	${INC_DIR}AppModels.h
	${INC_DIR}BaseModel.h
	${INC_DIR}LocalEnvironment.h
	${INC_DIR}PlanningModel.h
	${INC_DIR}PropertyModel.h
	${INC_DIR}SelectionModel.h
	${INC_DIR}VisualizationModel.h
	${INC_DIR}ProblemDefinitionModel.h
	${INC_DIR}WorldVisualizationModel.h
	)

set(Private_Includes
	${INC_DIR}private/muk.pch
	${INC_DIR}private/GoalAnalyzer.h
	${INC_DIR}private/PathEvaluator.h
	${INC_DIR}private/PropertyDetails.h
	${INC_DIR}private/VisualizationModelLegacyIO.h
	${INC_DIR}private/WorldVisualizationModelImpl.h
	)

set(Sources
	VisualizationModel.cpp
	AlgorithmModel.cpp
	ApplicationModel.cpp
	AppModels.cpp
	BaseModel.cpp
	LocalEnvironment.cpp
	PlanningModel.cpp
	PropertyDetails.cpp
	PropertyModel.cpp
	SelectionModel.cpp
	ProblemDefinitionModel.cpp
	WorldVisualizationModel.cpp
	
	# private
	GoalAnalyzer.cpp
	PathEvaluator.cpp
	VisualizationModelLegacyIO.cpp
	)

# configure the resource file
gris_configure_resource_rc(Resources MuknoII MukAppModels ${MODULE_NAME})

add_library(${PROJECT_NAME} SHARED	
	${Sources}
	${Includes}
	${Private_Includes}
	${Resources}
	)

# add more include directories
target_include_directories(${PROJECT_NAME} PUBLIC
	${INC_DIR}
	${GMP_INCLUDE_DIR}
	${EIGEN_INCLUDE_DIR}
	${CGAL_INCLUDE_DIR}
	)

include(${ITK_USE_FILE})
include(${VTK_USE_FILE})
	
target_link_libraries(${PROJECT_NAME} PUBLIC
	MukCommon
	MukEvaluation
	MukImaging
	MukPathPlanning
	MukQt
	MukVisualization
	gstd
	${ITK_LIBRARIES}
	${VTK_LIBRARIES}
	${BOOST_TARGETS}
	)
  
source_group("Header Files\\private" FILES
	${Private_Includes}
	)
source_group("Resources" FILES
	${Resources}
	)

# ---------- deploy and install

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")

