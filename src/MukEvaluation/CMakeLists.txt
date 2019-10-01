# ------------------------------------------------------------------------------------------
project(MukEvaluation)
# ------------------------------------------------------------------------------------------

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/)
        
set(Includes
	${INC_DIR}CollisionDetectionHandler.h
	${INC_DIR}Experiment.h
	${INC_DIR}muk_evaluation_api.h
	${INC_DIR}SegmentationEvaluator.h
	${INC_DIR}statistics.h
	)
set(Private_Includes
	${INC_DIR}private/muk.pch
	)
set(Sources
	CollisionDetectionHandler.cpp
	Experiment.cpp
	muk.cpp
	SegmentationEvaluator.cpp
	statistics.cpp
	)

# ------------------------------------------------------------------------------------------
  
# configure the resource file
gris_configure_resource_rc(Resources MuknoII Evaluation ${PROJECT_NAME})

include(${VTK_USE_FILE})

# ------------------------------------------------------------------------------------------

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
	${QCUSTOMPLOT_INCLUDE_DIR}
	${ITK_INCLUDE_DIR}
	)

target_link_libraries(${PROJECT_NAME}
	MukCommon
	MukQt
	CGAL-vc140-mt-4.7
	qcustomplot
	gstd
	${VTK_LIBRARIES}
	${BOOST_TARGETS}
	)

# ------------------------------------------------------------------------------------------

source_group("Header Files\\private" FILES
	${Private_Includes}
	)
	
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${MODULE_NAME})

# ----------- deploy and install --------------------------------------------------------

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")