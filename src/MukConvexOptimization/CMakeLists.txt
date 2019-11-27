project(MukConvexOptimization)

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/)

set(Private_Includes
	${INC_DIR}private/muk.pch
	)
set(Includes
	${INC_DIR}/ConvexBezierSplineOptimizer.h
	${INC_DIR}/muk_convex_optimization_api.h
	${INC_DIR}/opt_types.h
	${INC_DIR}/PluginManagerConvexOptimization.h
	)
set(Includes_opt
 	${INC_DIR}opt/AffineExpression.h
 	${INC_DIR}opt/BasicTrustRegion.h
 	${INC_DIR}opt/Cnt.h
 	${INC_DIR}opt/Constraint.h
 	${INC_DIR}opt/ConvexConstraints.h
 	${INC_DIR}opt/ConvexObjective.h
 	${INC_DIR}opt/ConvexOptimizationSolver.h
 	${INC_DIR}opt/Cost.h
 	${INC_DIR}opt/FunctionConstraint.h
 	${INC_DIR}opt/GurobiSolver.h
 	${INC_DIR}opt/MatrixOfVector.h
 	${INC_DIR}opt/OptimizationProblem.h
 	${INC_DIR}opt/Optimizer.h
 	${INC_DIR}opt/opt_util.h
 	${INC_DIR}opt/QuadraticExpression.h
 	${INC_DIR}opt/Variable.h
 	${INC_DIR}opt/VectorToVectorFunction.h
	)
set(Includes_bezier
	${INC_DIR}bezier/bezier_types.h
	${INC_DIR}bezier/BezierConfig.h
	${INC_DIR}bezier/BezierOptimizationModel.h
	${INC_DIR}bezier/BezierCurvatureConstraint.h
	${INC_DIR}bezier/BezierDistanceConstraint.h
	${INC_DIR}bezier/BezierDistanceCost.h
	${INC_DIR}bezier/BezierLengthCost.h
	${INC_DIR}bezier/BezierOptimizationModel.h
	${INC_DIR}bezier/BezierSplineCurvatureConstraint.h
	${INC_DIR}bezier/BezierSplineCurvatureCost.h
	${INC_DIR}bezier/BezierSplineDistanceConstraint.h
	${INC_DIR}bezier/BezierSplineDistanceCost.h
	)

# ------------------------------------------------------------------------------------

set(Sources
	ConvexBezierSplineOptimizer.cpp
	PluginManagerConvexOptimization.cpp
	)
set(Sources_opt
 	opt/AffineExpression.cpp
 	opt/BasicTrustRegion.cpp
 	opt/Constraint.cpp
 	opt/ConvexConstraints.cpp
 	opt/ConvexObjective.cpp
 	opt/ConvexOptimizationSolver.cpp
 	opt/FunctionConstraint.cpp
 	opt/GurobiSolver.cpp
 	opt/MatrixOfVector.cpp
 	opt/OptimizationProblem.cpp
 	opt/Optimizer.cpp
 	opt/opt_util.cpp
 	opt/QuadraticExpression.cpp
 	opt/Variable.cpp
 	opt/VectorToVectorFunction.cpp
	)
set(Sources_bezier
	bezier/BezierConfig.cpp
	bezier/BezierCurvatureConstraint.cpp
	bezier/BezierDistanceConstraint.cpp
	bezier/BezierDistanceCost.cpp
	bezier/BezierLengthCost.cpp
	bezier/BezierOptimizationModel.cpp
	bezier/BezierSplineCurvatureConstraint.cpp
	bezier/BezierSplineCurvatureCost.cpp
	bezier/BezierSplineDistanceConstraint.cpp
	bezier/BezierSplineDistanceCost.cpp
	)

# ------------------------------------------------------------------------

# configure the resource file
gris_configure_resource_rc(Resources MuknoII ConvexOptimization ${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
	${Private_Includes}
	${Includes}
	${Includes_opt}
	${Includes_bezier}
	${Sources}
	${Sources_opt}
	${Sources_bezier}
	${Resources}
	)

include(${VTK_USE_FILE})
		
target_include_directories(${PROJECT_NAME} PUBLIC
	${INC_DIR}
	${EIGEN_INCLUDE_DIR}
	${GUROBI_INCLUDE_DIR}
	)

target_link_libraries(${PROJECT_NAME}
	MukCommon
	MukPathPlanning
	gstd
	${BOOST_TARGETS}
	${GUROBI_LIBRARIES}
	)

# ------------------------------------------------------------------------	

source_group("Header Files"				FILES ${Includes}			)
source_group("Header Files\\private"	FILES ${Private_Includes}	)
source_group("Header Files\\opt"		FILES ${Includes_opt}		)
source_group("Header Files\\bezier"		FILES ${Includes_bezier}	)

source_group("Source Files"				FILES ${Sources}		)
source_group("Source Files\\opt"		FILES ${Sources_opt}	)
source_group("Source Files\\bezier"		FILES ${Sources_bezier}	)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${MODULE_NAME})

# ---------- deploy and install
gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")