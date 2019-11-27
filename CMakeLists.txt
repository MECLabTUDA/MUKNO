cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

project(MUKNO2)

include("${CMAKE_SOURCE_DIR}/cmake/MuknoModuleMacros.cmake")

# ----------------------
# configuration option SDK_BASEPATH
# ----------------------
set(SDK_BASEPATH CACHE PATH "specifies the sdk base path")
if (SDK_BASEPATH STREQUAL "")
  message(FATAL_ERROR "Set SDK_BASEPATH to the directory containing the 3rd party libraries")
endif()

# ----------------------
# configure and find GrisFramework
# ----------------------
set(GrisFramework_Required_Version "0.6")
set(GrisFramework_DIR "${SDK_BASEPATH}/GrisFramework/lib/cmake/GrisFramework-${GrisFramework_Required_Version}" CACHE PATH "specifies the GrisFramework package directory")
find_package("GrisFramework" "${GrisFramework_Required_Version}" REQUIRED)

# ----------------------
# set the Version
# ----------------------
include(cmake/Version.cmake) # Version uses GrisFramework

# ----------------------
# set INSTALL dirs 
# ----------------------
set(MuknoII_INCLUDE_INSTALL_DIR "include/MuknoII-${MuknoII_SHORT_VERSION}")
set(MuknoII_PACKAGE_INSTALL_DIR "lib/cmake/MuknoII-${MuknoII_SHORT_VERSION}")
set(MuknoII_Testing_DIR "tests")

set(MuknoII_CMAKE_DIR      "${CMAKE_SOURCE_DIR}/cmake/")
set(MuknoII_BIN_DIR        "bin")
set(MuknoII_Resources_DIR  "resources")
set(MuknoII_Data_DIR       "data")
set(MuknoII_Doc_DIR        "doc")
set(MuknoII_Result_DIR     "results")

# ----------------------
# configure and find boost libraries
# ----------------------
set(BOOST_ROOT "${SDK_BASEPATH}/boost_1_59_0" CACHE PATH "Boost root directory")
set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib64-msvc-14.0"  CACHE PATH "Boost library directory")
mark_as_advanced(BOOST_LIBRARYDIR)
# configuration options for Boost
set(Boost_COMPONENTS
    "chrono" "date_time" "filesystem" "program_options" "regex"
    "serialization" "system" "thread" "unit_test_framework")

# gris_boost_find_and_fix populates BOOST_TARGETS with the imported targets
# as well as Boost::dynamic_linking for shared libs / dlls
gris_boost_find_and_fix("1.59" REQUIRED COMPONENTS ${Boost_COMPONENTS})
# ----------------------
# set configuration options for itk/vtk/qcustomplot/cgal/loki/ompl
# ----------------------
# VTK
SET(VTK_Required_Version "7.1.1")
SET(VTK_Required_SHORT_Version "7.1")
set(VTK_DIR "${SDK_BASEPATH}/VTK-${VTK_Required_Version}/lib/cmake/vtk-${VTK_Required_SHORT_Version}/" CACHE PATH "specifies the ITK build directory")

# add VTK_DIR to the CMAKE_PREFIX_PATH
if(NOT "${CMAKE_PREFIX_PATH}" MATCHES "lib[\\\\\\/]cmake[\\\\\\/]vtk\\-" AND VTK_DIR)
  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${VTK_DIR})
endif()

# ITK
SET(ITK_Required_Version "4.11")
SET(ITK_Required_SHORT_Version "4.11")
set(ITK_DIR "${SDK_BASEPATH}/InsightToolkit-${ITK_Required_Version}/lib/cmake/ITK-${ITK_Required_SHORT_Version}" CACHE PATH "specifies the VTK build directory")

# add ITK_DIR to the CMAKE_PREFIX_PATH
#if(NOT "${CMAKE_PREFIX_PATH}" MATCHES "lib[\\\\\\/]cmake[\\\\\\/]ITK\\-" AND ITK_DIR)
#  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${ITK_DIR})
#endif()

# QCUSTOMPLOT
set(QCUSTOMPLOT_INCLUDE_DIR "${SDK_BASEPATH}/qcustomplot-1.3.2/include/" CACHE PATH "specifies the qcustomplot include directory")
set(QCUSTOMPLOT_LIB_DIR "${SDK_BASEPATH}/qcustomplot-1.3.2/lib/" CACHE PATH "specifies the qcustomplot library directory")
set(QCUSTOMPLOT_BIN_DIR "${SDK_BASEPATH}/qcustomplot-1.3.2/bin/" CACHE PATH "specifies the qcustomplot binary directory")

# CGAL
set(CGAL_DIR "${SDK_BASEPATH}/CGAL-4.7/lib/CGAL/" CACHE PATH "specifies the CGAL cmake directory")
find_package(CGAL REQUIRED)
include( ${CGAL_USE_FILE} )

set(EIGEN_INCLUDE_DIR "${SDK_BASEPATH}/Eigen-3.2.7/include/eigen3/" CACHE PATH "specifies the Eigen include directory necessary for CGAL")
set(GMP_DIR "${SDK_BASEPATH}/gmp/" CACHE PATH "specifies the gmp directory necessary for CGAL")
set(GMP_INCLUDE_DIR "${GMP_DIR}/include/")
set(GMP_BIN_DIR "${GMP_DIR}/lib/")

# LOKI
find_package(LOKI)

# OMPL
#set(OMPL_DIR "${SDK_BASEPATH}/ompl-1.2.1/share/ompl/" CACHE PATH "specifies the OMPL include directory")
#FIND_PACKAGE(ompl)
set(OMPL_INCLUDE_DIR "${SDK_BASEPATH}/OMPL-1.2.1/include/" CACHE PATH "specifies the OMPL include directory")
set(OMPL_LIB_DIR "${SDK_BASEPATH}/OMPL-1.2.1/lib/" CACHE PATH "specifies the OMPL library directory")
set(OMPL_BIN_DIR "${SDK_BASEPATH}/OMPL-1.2.1/bin/" CACHE PATH "specifies the OMPL binary directory")

# ACVD (surface remeshing)
set(ACVD_DIR "${SDK_BASEPATH}/ACVD_vtk-7.1.1/lib/cmake/ACVD/" CACHE PATH "specifies the ACVD include directory")
find_package(ACVD)

# ----------------------
# find previously configured libraries
# ----------------------
find_package( VTK ${VTK_Required_SHORT_Version}  REQUIRED )
if(VTK_FOUND)
  if(NOT CMAKE_PREFIX_PATH OR NOT ${VTK_DIR} IN_LIST CMAKE_PREFIX_PATH)
    set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${VTK_DIR}" CACHE PATH "Additional paths to look in for other modules/packages")
  endif()
  find_package( ITK ${ITK_Required_SHORT_Version} REQUIRED)
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ----------------------
# find and configure Qt
# ----------------------
find_package(Qt5Widgets 5.5 REQUIRED)
find_package(Qt5PrintSupport 5.5 REQUIRED)
if(${Qt5Widgets_FOUND})
  get_filename_component(_QT_DIR_TEMP "${Qt5Widgets_DIR}" PATH)
  get_filename_component(_QT_DIR_TEMP "${_QT_DIR_TEMP}" PATH)
  get_filename_component(QT_DIR "${_QT_DIR_TEMP}" PATH)
endif()

# ----------------------
# set optional libraries and applications that use convex optimization (and therefore the Gurobi Library)
# ----------------------
option(MukLibs_ConvexOptimization "Library that provides convex optimization algorithms" OFF)
if (${MukLibs_ConvexOptimization})
  find_package(GUROBI REQUIRED)
  if(NOT ${GUROBI_FOUND})
    message(FATAL_ERROR "Gurobi not found.")
  endif()
  gris_bundle_add_lookup_directories(${GUROBI_BIN_DIR})
endif()

# ----------------------
# set optional libraries and applications that use the tensorflow C-API
# ----------------------
option(MukLibs_TensorflowCAPI "use the tensorflow plugin with its C-API" OFF)
if (${MukLibs_TensorflowCAPI})
  find_package(TensorflowCAPI REQUIRED)
  if(NOT ${TensorflowCAPI_FOUND})
    message(FATAL_ERROR "Tensorflow C-API not found.")
  endif()
endif()


#Use solution folders.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include_directories(
	${CMAKE_SOURCE_DIR}/include/
	)

# add more link directories
link_directories(
	${OMPL_LIB_DIR}
	${QCUSTOMPLOT_LIB_DIR}
)

add_subdirectory(src)

gris_bundle_configure_file()
