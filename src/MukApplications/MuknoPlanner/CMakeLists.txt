project(MuknoPlanner)

include(ConfigureVersion)

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${MODULE_NAME}/${PROJECT_NAME}/)
set(MODEL_DIR ${CMAKE_SOURCE_DIR}/include/MukAppModels/)

set(Includes
	${INC_DIR}/AppControllers.h
	${INC_DIR}/AlgorithmController.h
	${INC_DIR}/BaseController.h
	${INC_DIR}/InteractionController.h
	${INC_DIR}/MenuBarController.h
	${INC_DIR}/MuknoPlanner.h
	${INC_DIR}/NavigationThread.h
	${INC_DIR}/NavigationWorker.h
	${INC_DIR}/PlanningController.h
	${INC_DIR}/ProblemDefinitionController.h
	${INC_DIR}/PropertyController.h
	${INC_DIR}/SafeApplication.h
	${INC_DIR}/SafeProperties.h
	${INC_DIR}/SelectionController.h
	${INC_DIR}/ThreadedController.h
	${INC_DIR}/ToolBarController.h
	${INC_DIR}/VisualizationController.h
	)
set(Includes_Private
	${INC_DIR}private/DefaultInteraction3D.h
	${INC_DIR}private/HandlerTextEditLog.h
	${INC_DIR}private/IMukInteraction.h
	${INC_DIR}private/muk.pch
	${INC_DIR}private/PlaneRegionInteraction.h
	${INC_DIR}private/RegionMarker.h
	${INC_DIR}private/RegisterQtMetaTypes.hxx	
	${INC_DIR}private/SphereRegionInteraction.h
	${INC_DIR}private/SurfaceRegionInteraction.h
	${INC_DIR}private/MultiPortSphereRegionInteraction.h
	${INC_DIR}private/SurfaceCutter.h
	)
set(Sources
	${PROJECT_NAME}.cpp	
	AppControllers.cpp
	AlgorithmController.cpp
	BaseController.cpp
	InteractionController.cpp
	
	main.cpp
	MenuBarController.cpp
	NavigationThread.cpp
	NavigationWorker.cpp
	PlanningController.cpp
	ProblemDefinitionController.cpp
	PropertyController.cpp
	SafeApplication.cpp
	SafeProperties.cpp
	SelectionController.cpp
	ToolBarController.cpp
	VisualizationController.cpp

	#private
	DefaultInteraction3D.cpp
	HandlerTextEditLog.cpp
	IMukInteraction.cpp
	MultiPortSphereRegionInteraction.cpp
	PlaneRegionInteraction.cpp
	RegionMarker.cpp
	SphereRegionInteraction.cpp
	SurfaceCutter.cpp
	SurfaceRegionInteraction.cpp
  )
  
# configure the resource file
gris_configure_resource_rc(Resources MuknoII Planner ${MODULE_NAME})

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}	${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
# Visual studio does not recognize option -fPIC
string(REPLACE "-fPIC" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

add_executable(${PROJECT_NAME}
	${Includes_Private}
	${Includes}
	${Sources}
	${Resources}
	${Files}
	)

include(${ITK_USE_FILE})
include(${VTK_USE_FILE})

target_include_directories(${PROJECT_NAME} PUBLIC
	${INC_DIR}	
	${EIGEN_INCLUDE_DIR}
	${GMP_INCLUDE_DIR}
	${CGAL_INCLUDE_DIR}	
)
	
target_link_libraries(${PROJECT_NAME}
	MukAlgorithms
	MukAppModels
	MukCommon
	MukEvaluation
	MukImaging
	MukNavigation
	MukPathPlanning
	MukVisualization
	MukQt
	gstd
	CGAL-vc140-mt-4.7
	QCustomPlot
	${BOOST_TARGETS}
	${Qt5Widgets_LIBRARIES}
	)

## CONDITIONALLY Linked libraries
if(${MukLibs_ConvexOptimization})
  target_link_libraries(${PROJECT_NAME}    MukConvexOptimization    )
  target_compile_definitions(${PROJECT_NAME} PUBLIC      MUK_LINK_CONVEX_OPTIMIZATION    )
endif()

if(${MukLibs_TensorflowCAPI})
  target_link_libraries(${PROJECT_NAME} 
	MukTensorflowCAPI
  )
  target_compile_definitions(${PROJECT_NAME} PUBLIC      MUK_LINK_TENSORFLOW_CAPI    )
endif()

# prepare Versions file
gris_version_configure(${PROJECT_NAME})

# in this folder, the file requiredLibraryVersions.hxx-file can be found
target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
  
source_group("Header Files\\private" FILES
	${Includes_Private}
	)

source_group("resources" FILES
  ${Resources}
	)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${MODULE_NAME})

# ---------- post build 

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")
# docs currently do not exist
#gris_deploy_directories(${PROJECT_NAME} "doc/${MODULE_NAME}/${PROJECT_NAME}" "${CMAKE_SOURCE_DIR}/doc/${MODULE_NAME}/${PROJECT_NAME}")
# add binary directories, that have to be manually added (libraries using dlls, that are not targets)
# --> Boost has targets, but dynamic linking messes with that
# --> GMP is required through CGAL, but has no CMake Targets)

set(Bin_Lookup_Directories 
	${QCUSTOMPLOT_BIN_DIR}	
	${Boost_LIBRARY_DIRS}
	${GMP_BIN_DIR}
	)

gris_bundle_add_lookup_directories(${Bin_Lookup_Directories})
gris_bundle(${PROJECT_NAME})

