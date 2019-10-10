project(MukVisualization)

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/)

set(Includes
	${INC_DIR}/ArrowTrajectory.h
	${INC_DIR}/BasicVisualization.h
	${INC_DIR}/CameraConfiguration.h
	${INC_DIR}/Cursor3DRepresentation.h
	${INC_DIR}/Cursor3DWidget.h
	${INC_DIR}/IVisRobot.h
	${INC_DIR}/muk_colors.h
	${INC_DIR}/muk_visualization_api.h
	${INC_DIR}/MukRobotSource.h
	${INC_DIR}/PlaneRegionRepresentation.h
	${INC_DIR}/PlaneRegionWidget.h
	${INC_DIR}/PolyDataHandler.h
	${INC_DIR}/PolyDataMapperHandler.h
	${INC_DIR}/ROIRepresentation.h
	${INC_DIR}/ROISynchronizer.h
	${INC_DIR}/ROIWidget.h
	${INC_DIR}/SliceInteractorStyle.h
	${INC_DIR}/SliceRenderGroup.h
	${INC_DIR}/SphereRegionRepresentation.h
	${INC_DIR}/SphereRegionWidget.h
	${INC_DIR}/SurfaceRegionRepresentation.h
	${INC_DIR}/SurfaceRegionWidget.h
	${INC_DIR}/Vis3dPlane.h
	${INC_DIR}/VisAbstractObject.h
	${INC_DIR}/VisDistanceLine.h
	${INC_DIR}/VisAxes.h
	${INC_DIR}/VisBounds.h
	${INC_DIR}/VisCoordinateSystem.h
	${INC_DIR}/VisCoordinateSystemCollection.h
	${INC_DIR}/VisCursor3D.h
	${INC_DIR}/VisImageOverlay.h
	${INC_DIR}/MultiPortSphereRegionRepresentation.h
	${INC_DIR}/VisMukPath.h
	${INC_DIR}/VisMukPathGraph.h
	${INC_DIR}/VisMukProblemDefinition.h
	${INC_DIR}/VisMukRobotSource.h
	${INC_DIR}/MukVtkAxesActor.h	
	${INC_DIR}/VisObstacle.h
	${INC_DIR}/VisPathCollection.h
	${INC_DIR}/VisPlaneRegion.h
	${INC_DIR}/VisPreliminaryTestRobot.h
	${INC_DIR}/VisScaleableObject.h
	${INC_DIR}/VisScene.h
	${INC_DIR}/VisSE3Trajectory.h
	${INC_DIR}/VisSphereRegion.h
	${INC_DIR}/VisMultiPortSphereRegion.h
	${INC_DIR}/VisStateRegion.h
	${INC_DIR}/VisStateRegionFactory.h
	${INC_DIR}/VisSurfaceRegion.h
	${INC_DIR}/VisTimeDelays.h
	${INC_DIR}/VisTrajectory.h
	${INC_DIR}/visualization_tools.h
	${INC_DIR}/VisualObject.h
	${INC_DIR}/VisualProp.h
	${INC_DIR}/VisVector.h
	${INC_DIR}/VisWaypoints.h
	)

set(Private_Includes
	${INC_DIR}private/ArrowMaker.h
	${INC_DIR}private/ImageOverlayCallback.h
	${INC_DIR}private/muk.pch
	${INC_DIR}private/SurfaceRegionRepresentationCallback.h
	${INC_DIR}private/VisSceneLegacyIO.h
	)

set(Sources
	ArrowTrajectory.cpp
	BasicVisualization.cpp
	CameraConfiguration.cpp
	Cursor3DRepresentation.cpp
	Cursor3DWidget.cpp
	IVisRobot.cpp
	muk_colors.cpp
	MukRobotSource.cpp
	MukVtkAxesActor.cpp
	MultiPortSphereRegionRepresentation.cpp
	PlaneRegionRepresentation.cpp
	PlaneRegionWidget.cpp
	PolyDataHandler.cpp
	PolyDataMapperHandler.cpp
	ROIRepresentation.cpp
	ROISynchronizer.cpp
	ROIWidget.cpp
	SliceInteractorStyle.cpp
	SliceRenderGroup.cpp
	SphereRegionRepresentation.cpp
	SphereRegionWidget.cpp
	SurfaceRegionRepresentation.cpp
	SurfaceRegionWidget.cpp
	Vis3dPlane.cpp
	VisAbstractObject.cpp
	VisAxes.cpp
	VisBounds.cpp
	VisCoordinateSystem.cpp
	VisCoordinateSystemCollection.cpp
	VisCursor3D.cpp
	VisDistanceLine.cpp
	VisImageOverlay.cpp
	VisMukPathGraph.cpp
	VisMukPath.cpp
	VisMukProblemDefinition.cpp
	VisMukRobotSource.cpp
	VisMultiPortSphereRegion.cpp
	VisObstacle.cpp
	VisPathCollection.cpp
	VisPlaneRegion.cpp
	VisPreliminaryTestRobot.cpp
	VisScaleableObject.cpp
	VisScene.cpp
	VisSE3Trajectory.cpp
	VisSphereRegion.cpp
	VisStateRegion.cpp
	VisStateRegionFactory.cpp
	VisSurfaceRegion.cpp
	VisTimeDelays.cpp
	VisTrajectory.cpp
	visualization_tools.cpp
	VisualObject.cpp
	VisualProp.cpp
	VisVector.cpp
	VisWaypoints.cpp
	
	#private
	ArrowMaker.cpp
	ImageOverlayCallback.cpp
	muk.cpp
	SurfaceRegionRepresentationCallback.cpp
	VisSceneLegacyIO.cpp
	)

# configure the resource file
gris_configure_resource_rc(Resources MuknoII Visualization ${MODULE_NAME})

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
	)

include(${VTK_USE_FILE})
	
target_link_libraries(${PROJECT_NAME}
	gstd
	MukCommon
	${VTK_LIBRARIES}
	)
	
source_group("Header Files\\private" FILES
	${Private_Includes}
	)

# ---------- deploy and install

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")

