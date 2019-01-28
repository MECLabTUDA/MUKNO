project(MukEvaluation)

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/)

set(Includes
	${INC_DIR}CollisionDetectionHandler.h
	${INC_DIR}muk_evaluation_api.h	
	${INC_DIR}statistics.h
	)
set(Private_Includes
	${INC_DIR}private/muk.pch
	)
set(Sources
	CollisionDetectionHandler.cpp
	muk.cpp	
	statistics.cpp
	)
  
# configure the resource file
gris_configure_resource_rc(Resources MuknoII Evaluation ${MODULE_NAME})

add_library(${PROJECT_NAME} SHARED 	
	${Private_Includes}
	${Includes}
	${Sources}
	${Resources}
	${Files}
	)

target_include_directories(${PROJECT_NAME} PUBLIC
	${INC_DIR}
	)

include(${VTK_USE_FILE})
	
target_link_libraries(${PROJECT_NAME}
	gstd
	MukCommon
	CGAL-vc140-mt-4.7
	${VTK_LIBRARIES}
	${BOOST_TARGETS}
	)
	
source_group("Header Files\\private" FILES
	${Private_Includes}
	)

# ---------- deploy and install

gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")

  
