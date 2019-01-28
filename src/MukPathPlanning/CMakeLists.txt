project(MukPathPlanning)

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/)


set(Includes
	${INC_DIR}BezierSpline.h
	${INC_DIR}muk_pathplanning_api.h
	${INC_DIR}InterpolatorBezierSpirals.h	
	${INC_DIR}InterpolatorLinear.h
	${INC_DIR}InterpolatorSpiralsYangEtAl.h
	${INC_DIR}OmplPlanner.h
	${INC_DIR}PlannerLinear.h
	${INC_DIR}PlannerManualLinear.h
	${INC_DIR}PlannerSplineBasedRRTconnect.h
	${INC_DIR}PluginManagerPP.h
	${INC_DIR}PrunerDummy.h
	${INC_DIR}PrunerYangSpiral.h
	${INC_DIR}QuickPlanningConfigurator.h
	)

set(Private_Includes
	${INC_DIR}private/BezierSpiralInterpolator.h
	${INC_DIR}private/BezierSpiral3DInterpolator.h
	${INC_DIR}private/IStateValidityChecker.h
	${INC_DIR}private/Motion.h
	${INC_DIR}private/MotionValidatorDefault.h
	${INC_DIR}private/muk.pch
	${INC_DIR}private/MukGoalStates.h
	${INC_DIR}private/MukOmplSetup.h
	${INC_DIR}private/MukGoalStates.h
	${INC_DIR}private/pathplanning_tools.h
	${INC_DIR}private/PlannerStatistic.h
	${INC_DIR}private/SplineBasedRRTconnect.h
	${INC_DIR}private/StateSamplerBiasedGreedy.h
	${INC_DIR}private/StateSamplerIBounds.h
	${INC_DIR}private/types.h
	${INC_DIR}private/ValidityCheckerDefault.h
	${INC_DIR}private/ValidityCheckerKdTreeNearestNeighbor.h
	)

set(Sources
	BezierSpline.cpp
	PluginManagerPP.cpp
	InterpolatorBezierSpirals.cpp
	InterpolatorLinear.cpp
	InterpolatorSpiralsYangEtAl.cpp
	OmplPlanner.cpp
	PlannerLinear.cpp
	PlannerManualLinear.cpp
	PlannerSplineBasedRRTconnect.cpp
	PluginManagerPP.cpp
	PrunerDummy.cpp
	PrunerYangSpiral.cpp
	ValidityCheckerKdTreeNearestNeighbor.cpp

	# privates
	BezierSpiralInterpolator.cpp
	BezierSpiral3DInterpolator.cpp
	IStateValidityChecker.cpp
	Motion.cpp
	muk.cpp
	MukGoalStates.cpp
	MukOmplSetup.cpp	
	MukGoalStates.cpp
	pathplanning_tools.cpp
	QuickPlanningConfigurator.cpp
	SplineBasedRRTconnect.cpp
	StateSamplerBiasedGreedy.cpp
	StateSamplerIBounds.cpp
	ValidityCheckerKdTreeNearestNeighbor.cpp
	)

# configure the resource file
gris_configure_resource_rc(Resources MuknoII PathPlanning ${MODULE_NAME})

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
	${OMPL_INCLUDE_DIR}
	)
	
target_link_libraries(${PROJECT_NAME}
	gstd
	MukCommon
	${CGAL_LIBRARIES}
	ompl.lib
	${BOOST_TARGETS}
	)
	
source_group("Header Files\\private" FILES
	${Private_Includes}
	)

# ---------- deploy and install

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")
