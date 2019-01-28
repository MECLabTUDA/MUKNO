project(${LIB_NAME})

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${LIB_NAME}/)

set(Includes
# AlgorithmWrappers
	${INC_DIR}muk_algorithms_tools.h
	${INC_DIR}AndOrNot.h
	${INC_DIR}AlgorithmFactory.h
	${INC_DIR}AlgorithmManager.h
	${INC_DIR}AlgorithmWrapper.h
	${INC_DIR}And.h
	${INC_DIR}BinaryDilate.h
	${INC_DIR}BinaryErode.h
	${INC_DIR}BinaryNot.h
	${INC_DIR}BinaryThresholdFilter.h
	${INC_DIR}CannyEdgeDetectionImageFilter.h
	${INC_DIR}ConfidenceConnectedImageFilter.h
	${INC_DIR}ConnectedThresholdImageFilter.h
	${INC_DIR}CurvatureAnisotropicDiffusionImageFilter.h
	${INC_DIR}CurvatureFlowImageFilter.h
	${INC_DIR}DeformableMesh.h
	${INC_DIR}DiscreteGaussian.h
	${INC_DIR}DistanceMap.h
	${INC_DIR}GradientImageFilter.h
	${INC_DIR}FlipImage.h
	${INC_DIR}DiscreteGaussianImageFilter.h
	${INC_DIR}GradientAnisotropicDiffusionImageFilter.h
	${INC_DIR}ImageFileWriter.h
	${INC_DIR}ImageReader.h
	${INC_DIR}ITKWrapper.h
	${INC_DIR}LaplacianImageFilter.h
	${INC_DIR}LargestRegion.h
	${INC_DIR}MarchingCubes.h
	${INC_DIR}MedianImageFilter.h
	${INC_DIR}MeshReduction.h
	${INC_DIR}MeshSource.h
	${INC_DIR}meshToBinaryImage.h
	${INC_DIR}MeshToImageFilter.h
	${INC_DIR}MorphologicalClosing.h
	${INC_DIR}muk_algorithms_api.h
	${INC_DIR}Or.h
	${INC_DIR}OpenClose3D.h
	${INC_DIR}OtsuThresholding.h
	${INC_DIR}PointCloudFromImage.h
	${INC_DIR}PolyDataReader.h
	${INC_DIR}PolyDataWriter.h
	${INC_DIR}RescaleFilter.h
	${INC_DIR}RegionOfInterest.h
	${INC_DIR}Remesher.h
	${INC_DIR}RescaleIntensity.h
	${INC_DIR}SobelEdgeDetectionImageFilter.h
	${INC_DIR}Skeletonization.h
	${INC_DIR}Skeletonization2.h
	${INC_DIR}SmoothSurfaceExtraction.h
	${INC_DIR}SobelFilter.h
	${INC_DIR}ThresholdFilter.h
	${INC_DIR}VTKWrapper.h
	## new 2D filter
	${INC_DIR}SliceImageFileWriter.h
	${INC_DIR}ConnectedThresholdSliceImageFilter.h
	${INC_DIR}OrSliceImage.h
	${INC_DIR}BinaryThresholdSliceFilter.h
	${INC_DIR}GaussianSliceFilter.h
	${INC_DIR}WatershedSliceFilter.h
	${INC_DIR}SliceImageReader.h
	${INC_DIR}MedianSliceImageFilter.h
	${INC_DIR}FloatSliceImageToSliceImageFilter.h
	)

set(Private_Includes
	${INC_DIR}private/LargestRegionFilter.h
	${INC_DIR}private/muk.pch
	${INC_DIR}private/SkeletonizationFilter.h
	${INC_DIR}private/SkeletonizationFilter.hxx
	${INC_DIR}private/tools.h	
	)

set(Sources
	AlgorithmFactory.cpp
	AlgorithmManager.cpp
	AlgorithmWrapper.cpp
	And.cpp
	AndOrNot.cpp
	BinaryDilate.cpp
	BinaryErode.cpp
	BinaryNot.cpp
	BinaryThresholdFilter.cpp
	CannyEdgeDetectionImageFilter.cpp
	ConfidenceConnectedImageFilter.cpp
	ConnectedThresholdImageFilter.cpp
	CurvatureAnisotropicDiffusionImageFilter.cpp
	CurvatureFlowImageFilter.cpp
	DeformableMesh.cpp
	DiscreteGaussian.cpp
	DistanceMap.cpp
	FlipImage.cpp
	GradientImageFilter.cpp
	DiscreteGaussianImageFilter.cpp
	GradientAnisotropicDiffusionImageFilter.cpp
	ImageFileWriter.cpp
	ImageReader.cpp
	LaplacianImageFilter.cpp
	LargestRegion.cpp
	LargestRegionFilter.cpp
	MarchingCubes.cpp
	MedianImageFilter.cpp
	Or.cpp
	RescaleFilter.cpp
	SobelEdgeDetectionImageFilter.cpp
	MeshReduction.cpp
	MeshSource.cpp
	meshToBinaryImage.cpp
	MeshToImageFilter.cpp
	MorphologicalClosing.cpp
	muk_algorithms_tools.cpp
	
	PointCloudFromImage.cpp
	PolyDataReader.cpp
	PolyDataWriter.cpp
	OpenClose3D.cpp
	OtsuThresholding.cpp
	RegionOfInterest.cpp
	Remesher.cpp
	RescaleIntensity.cpp
	SmoothSurfaceExtraction.cpp
	SobelFilter.cpp
	ThresholdFilter.cpp
	SliceImageFileWriter.cpp
	ConnectedThresholdSliceImageFilter.cpp
	OrSliceImage.cpp
	BinaryThresholdSliceFilter.cpp
	GaussianSliceFilter.cpp
	WatershedSliceFilter.cpp
	SliceImageReader.cpp
	MedianSliceImageFilter.cpp
	FloatSliceImageToSliceImageFilter.cpp
	
	#private
	Skeletonization.cpp
	Skeletonization2.cpp
	tools.cpp
	)
  
# configure the resource file
gris_configure_resource_rc(Resources MuknoII Algorithms ${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
	${Private_Includes}
	${Includes}
	${Sources}
	${Resources}
	${Files}	
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
	gstd
	MukCommon
	MukImaging
	${ITK_LIBRARIES}
	${VTK_LIBRARIES}
	vtkDiscreteRemeshing
	vtkSurface
	vtkVolumeProcessing
	${ACVDTargets}
	${BOOST_TARGETS}
	)
	
	target_link_libraries(${PROJECT_NAME} PRIVATE	
	LOKI	
	)
	
source_group("Header Files\\private" FILES
	${Private_Includes}
	)
	
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${MODULE_NAME})
	
# ---------- deploy and install

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")