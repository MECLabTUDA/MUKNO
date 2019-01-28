project(MukCommon)
	
set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${MODULE_NAME}/)

set(Includes
	${INC_DIR}Bounds.h
    ${INC_DIR}CalibrationFactory.h
    ${INC_DIR}CalibrationStateMachine.h
	${INC_DIR}CollisionDetectorKdTree.h
	${INC_DIR}EvaluationFactory.h
	${INC_DIR}geometry.h
	${INC_DIR}gris_math.h
	${INC_DIR}gris_property_streams.h
	${INC_DIR}MukItkIO.h
	${INC_DIR}MukItkIO.h
	${INC_DIR}IAbstractFactory.h
	${INC_DIR}IBounds.h
	${INC_DIR}ICollisionDetector.h	
	${INC_DIR}IInterpolator.h
	${INC_DIR}IMukEvaluation.h
	${INC_DIR}INavigator.h
	${INC_DIR}INavigationContainer.h
	${INC_DIR}INavigationSupervisor.h
	${INC_DIR}InterpolatorFactory.h
	${INC_DIR}IPathPlanner.h
	${INC_DIR}IPathPruner.h
	${INC_DIR}LoggerWrapper.h
	${INC_DIR}IStateRegion.h
	${INC_DIR}muk_common_api.h
	${INC_DIR}muk_common.h	
	${INC_DIR}muk_eigen_helper.h
	${INC_DIR}MukException.h
	${INC_DIR}MukIO.h
	${INC_DIR}MukObstacle.h	
	${INC_DIR}MukPath.h
	${INC_DIR}MukPathGraph.h
	${INC_DIR}MukProblemDefinition.h
	${INC_DIR}MukScene.h
	${INC_DIR}MukState.h
	${INC_DIR}MukStateRegion.h
	${INC_DIR}MukTransform.h
	${INC_DIR}MukStringToolkit.h
	${INC_DIR}MukTransform.h
	${INC_DIR}MukVector.h
	${INC_DIR}MultiPortStateRegion.h
	${INC_DIR}NavigatorFactory.h
	${INC_DIR}NewtonRaphsonMethod.h
	${INC_DIR}NewtonRaphsonMethod.hxx
	${INC_DIR}overload_deduction.h
	${INC_DIR}PathCollection.h
	${INC_DIR}PlannerFactory.h
	${INC_DIR}PlaneStateRegion.h
	${INC_DIR}polygon_3d_tools.h
	${INC_DIR}PrunerFactory.h
	${INC_DIR}StateRegionFactory.h
	${INC_DIR}StateRegionSingleDirection.h
	${INC_DIR}StateSamplerData.h
	${INC_DIR}StreamReader.h
	${INC_DIR}SurfaceStateRegion.h
	${INC_DIR}SystemCalibration.h
    ${INC_DIR}TimerOwner.h
	${INC_DIR}version.h
	${INC_DIR}Waypoints.h
	${INC_DIR}Typedefs.h
	)

set(Private_Includes
	${INC_DIR}private/muk.pch
	${INC_DIR}private/MukSceneLegacyIO.h
	)

set(Sources
	Bounds.cpp	
	CalibrationFactory.cpp
	CalibrationStateMachine.cpp
	CollisionDetectorKdTree.cpp	
	EvaluationFactory.cpp
	geometry.cpp
	gris_property_streams.cpp
	IBounds.cpp
	IInterpolator.cpp
	IMukEvaluation.cpp	
#  INavigationSupervisor.cpp
	INavigator.cpp
	InterpolatorFactory.cpp		
	IPathPlanner.cpp
	IPathPruner.cpp
	LoggerWrapper.cpp
	IStateRegion.cpp
	MukException.cpp
	MukIO.cpp
	MukItkIO.cpp
	MukObstacle.cpp
	MukPath.cpp
	MukPathGraph.cpp
	MukProblemDefinition.cpp
	MukScene.cpp
	MukSceneLegacyIO.cpp
	MukState.cpp
	MukStateRegion.cpp
	MukStringToolkit.cpp
    MukTransform.cpp
	MultiPortStateRegion.cpp
	NavigatorFactory.cpp
	PathCollection.cpp	
	PlannerFactory.cpp
	PlaneStateRegion.cpp
	polygon_3d_tools.cpp
	PrunerFactory.cpp
	StateRegionFactory.cpp
	StateRegionSingleDirection.cpp
	StateSamplerData.cpp
	StreamReader.cpp
	SurfaceStateRegion.cpp
	SystemCalibration.cpp
    TimerOwner.cpp
	version.cpp
	Waypoints.cpp
	)
  
# configure the resource file
gris_configure_resource_rc(Resources MuknoII Algorithms ${MODULE_NAME})

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
	version # windows dll for analysing file info
	gstd
	${CGAL_LIBRARIES}
	${ITK_LIBRARIES}
	${VTK_LIBRARIES}
	${BOOST_TARGETS}
	)

target_link_libraries(${PROJECT_NAME} PRIVATE
  LOKI)
  
source_group("Header Files\\private" FILES
	${Private_Includes}
	)
source_group("Resources" FILES
	${Resources}
	)

# ---------- deploy and install

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")

