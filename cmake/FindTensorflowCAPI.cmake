# setting the TensorflowCAPI properties for CMake modules
#
# FindTensorflowCAPI will export an imported library interface target to link to
#
# TensorflowCAPI_DIR is the path to the TensorflowCAPI include path
#
# Exported Target TensorflowCAPI


# define TensorflowCAPI_DIR as an "CACHE VARIABLE" and initialize

if(NOT TensorflowCAPI_DIR)
  if(DEFINED SDK_BASEPATH)
    set(_SDK_BASEPATH "${SDK_BASEPATH}")
  else()
    set(_SDK_BASEPATH "${CMAKE_INSTALL_PREFIX}")
  endif()
  set(TensorflowCAPI_DIR "${_SDK_BASEPATH}/libtensorflow-gpu-windows-x68_64-1.11.0" CACHE PATH "Specifies the TensorFlow C-API include directory")
endif()

	message(STATUS "searching Tensorflow C-API in ${TensorflowCAPI_DIR}")

#if(NOT TARGET TensorflowCAPI AND TensorflowCAPI_DIR)
  # check, whether all files are there
  set(TensorflowCAPI_Include_Dir "${TensorflowCAPI_DIR}/include/")
  set(TensorflowCAPI_Bin_Dir "${TensorflowCAPI_DIR}/bin/")
  set(TensorflowCAPI_Lib_Dir "${TensorflowCAPI_DIR}/lib/")
  set(TensorflowCAPI_Libaries "tensorflow")
  set(TensorflowCAPI_IN_DIR ON)
  set(TensorflowCAPI_FOUND ON)

  # Test if Include dir is complete
  if(NOT EXISTS "${TensorflowCAPI_Include_Dir}tensorflow/c_api.h")
    set(_TensorflowCAPI_IN_DIR OFF)
    set(TensorflowCAPI_FOUND   OFF)
    message(STATUS "c_api.h missing for TensorflowCAPI in TensorflowCAPI_DIR: ${TensorflowCAPI_Include_Dir}/tensorflow/")
  endif()
  # Test if Bin dir is complete
  if(NOT EXISTS "${TensorflowCAPI_Bin_Dir}tensorflow.dll")
    set(TensorflowCAPI_IN_DIR OFF)
    set(TensorflowCAPI_FOUND   OFF)
    message(STATUS "tensorflow.dll missing for TensorflowCAPI in TensorflowCAPI_DIR: ${TensorflowCAPI_Bin_Dir}")
  endif()
  # Test if Lib dir is complete
  foreach(_fileEnd .lib)
    if(NOT EXISTS "${TensorflowCAPI_Lib_Dir}tensorflow${_fileEnd}")
    set(TensorflowCAPI_IN_DIR OFF)
    set(TensorflowCAPI_FOUND   OFF)
      message(STATUS "tensorflow${_fileEnd} missing for TensorflowCAPI in TensorflowCAPI_DIR: ${TensorflowCAPI_Lib_Dir}")
    endif()
  endforeach()

  #add_library(TensorflowCAPI INTERFACE IMPORTED)

  #set_target_properties(TensorflowCAPI PROPERTIES
  #  INTERFACE_INCLUDE_DIRECTORIES "${TensorflowCAPI_DIR}"
  #)
  if(NOT ${TensorflowCAPI_IN_DIR})
    message("TensorflowCAPI could not be found")
  endif()
#endif()