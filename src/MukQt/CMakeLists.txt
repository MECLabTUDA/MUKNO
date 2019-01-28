project(MukQt)

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/)

set(Includes		
	${INC_DIR}/Cursor3DSynchronizer.h
	${INC_DIR}/MedicalMultiViewWidget.h
	${INC_DIR}/muk_qt_api.h	
	${INC_DIR}/muk_qt_tools.h
	${INC_DIR}/SliceWidget.h
	${INC_DIR}/VolumeWidget.h
	${INC_DIR}/VtkWindow.h
	)

set(Private_Includes
	${INC_DIR}private/muk.pch
	${INC_DIR}private/SliceSynchronizer.h
	${INC_DIR}private/SliceInteractorStyle.h
	)

set(Sources
	Cursor3DSynchronizer.cpp
	MedicalMultiViewWidget.cpp
	muk.cpp	
	muk_qt_tools.cpp
	VolumeWidget.cpp
	SliceWidget.cpp
	VtkWindow.cpp
	)
  
# configure the resource file
gris_configure_resource_rc(Resources MuknoII Qt ${MODULE_NAME})

# QT stuff
set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON) # this is only necessary if qt designer is used
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5PrintSupport_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}	${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
# Visual studio does not recognize option -fPIC
string(REPLACE "-fPIC" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

add_library(${PROJECT_NAME} SHARED 	
	${Private_Includes}
	${Includes}
	${Sources}
	${Resources}
	${Files}	
	)

target_include_directories(${PROJECT_NAME} PUBLIC
	${INC_DIR}	
	${CGAL_INCLUDE_DIR}	
	${EIGEN_INCLUDE_DIR}	
	${QCUSTOMPLOT_INCLUDE_DIR}
	)	
	
include(${VTK_USE_FILE})
include(${ITK_USE_FILE})

# define which Qt Modules to use
set(Muk_Qt_Modules Qt5::Widgets Qt5::PrintSupport)

target_link_libraries(${PROJECT_NAME}
	gstd
	MukAlgorithms
	MukCommon
	MukPathPlanning
	MukVisualization
	${VTK_LIBRARIES}
	${BOOST_TARGETS}
	${Muk_Qt_Modules}
	)
	
source_group("Header Files\\private" FILES
	${Private_Includes}
	)
  
# usage of Qt with cmake: http://doc.qt.io/qt-5/cmake-manual.html
# qt5_use_modules(${PROJECT_NAME} Widgets) --> this is for cmake < 2.8 
add_definitions(-DMUK_QT_CREATE_DLL)

# ---------- deploy and install

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")

gris_deploy_files(${PROJECT_NAME} "${MuknoII_BIN_DIR}/platforms" "${QT_DIR}/plugins/platforms/qwindows.dll")
