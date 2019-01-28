set(MODULE_NAME MukCommon)
add_subdirectory(${MODULE_NAME})
set_target_properties(${MODULE_NAME} PROPERTIES FOLDER Libraries)

set(MODULE_NAME Libraries)
set(LIB_NAME MukAlgorithms)
add_subdirectory(${LIB_NAME})

set(MODULE_NAME MukImaging)
add_subdirectory(${MODULE_NAME})
set_target_properties(${MODULE_NAME} PROPERTIES FOLDER Libraries)

set(MODULE_NAME MukPathPlanning)
add_subdirectory(${MODULE_NAME})
set_target_properties(${MODULE_NAME} PROPERTIES FOLDER Libraries)

set(MODULE_NAME MukEvaluation)
add_subdirectory(${MODULE_NAME})
set_target_properties(${MODULE_NAME} PROPERTIES FOLDER Libraries)

set(MODULE_NAME MukAppModels)
add_subdirectory(${MODULE_NAME})
set_target_properties(${MODULE_NAME} PROPERTIES FOLDER Libraries)

set(MODULE_NAME MukVisualization)
add_subdirectory(${MODULE_NAME})
set_target_properties(${MODULE_NAME} PROPERTIES FOLDER Libraries)