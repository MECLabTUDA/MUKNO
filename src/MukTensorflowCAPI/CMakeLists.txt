project(MukTensorflowCAPI)

set(INC_DIR ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/)

set(Private_Includes
	${INC_DIR}private/muk.pch
	${INC_DIR}private/ComputationalGraph.h
	)
set(Includes
	${INC_DIR}helper.h
	${INC_DIR}muk_tensorflow_c_api.h
	${INC_DIR}PluginManagerTensorflowCAPI.h
	${INC_DIR}TensorflowGraph.h
	)
set(Sources
	helper.cpp
	PluginManagerTensorflowCAPI.cpp
	TensorflowGraph.cpp

	# privates
	ComputationalGraph.cpp
	)
	
# ------------------------------------------------------------------------------------------------
  
# configure the resource file
gris_configure_resource_rc(Resources MuknoII TensorflowCAPI ${PROJECT_NAME})

link_directories(${TensorflowCAPI_Lib_Dir})
# ------------------------------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
	${Private_Includes}
	${Includes}
	${Sources}
	${Resources}
	)	
	
include(${VTK_USE_FILE})
include(${ITK_USE_FILE})

target_include_directories(${PROJECT_NAME} PUBLIC
	${INC_DIR}
	)

target_include_directories(${PROJECT_NAME} PRIVATE
	${TensorflowCAPI_Include_Dir}
	)
	
target_link_libraries(${PROJECT_NAME} PUBLIC
	MukAlgorithms
	MukAppModels
	MukCommon
	MukImaging
	gstd
	${VTK_LIBRARIES}
	)

target_link_libraries(${PROJECT_NAME} PRIVATE
	${TensorflowCAPI_Libaries}
	)
			
source_group("Header Files\\private" FILES
	${Private_Includes}
	)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ${MODULE_NAME})

# ---------- post build 
gris_deploy(${PROJECT_NAME} "${MuknoII_BIN_DIR}" "")
gris_bundle(${PROJECT_NAME})