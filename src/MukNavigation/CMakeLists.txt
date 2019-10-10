project(MukNavigation)

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/)

set(Includes
	${INC_DIR}CalibratorDummy.h
    ${INC_DIR}muk_navigation_api.h
	${INC_DIR}NavigatorDummy.h
	${INC_DIR}NavigatorNoisyReplanning.h
    ${INC_DIR}PluginManagerNavi.h
  )
set(Private_Includes
	${INC_DIR}private/muk.pch
	)
set(Sources
	CalibratorDummy.cpp
	muk.cpp
	NavigatorDummy.cpp
	NavigatorNoisyReplanning.cpp
	PluginManagerNavi.cpp
  )
  
# configure the resource file
gris_configure_resource_rc(Resources MuknoII Navigation ${MODULE_NAME})

set(ResourceFiles )
 
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