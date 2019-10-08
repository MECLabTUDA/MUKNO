option(MukUtilities_MuknoPlanner "Enable the Main Application for preoperative planning" OFF)
if (${MukUtilities_MuknoPlanner})
	add_subdirectory(MuknoPlanner)
endif()


option(MukUtilities_IPCAI2019 "Enable the Application for IPCAI 2019 conference" OFF)
if (${MukUtilities_IPCAI2019})
	add_subdirectory(Conf_IPCAI2019)
endif()

option(MukUtilities_MICCAI2019 "Enable the experiment application for MICCAI 2019 conference" OFF)
if (${MukUtilities_MICCAI2019})
	add_subdirectory(Conf_MICCAI2019)
endif()

option(MukUtilities_IROS2019 "Enable the experiment application for the RA-L paper 'Planning for Flexible Surgical Robots via Bézier Spline Translation Johannes'" OFF)
if (${MukUtilities_IROS2019})
	add_subdirectory(Conf_IROS2019)
endif()

option(MukUtilities_MICCAI2019_OR20 "Enable the Application for MICCAI 2019 OR 2.0 workshop conference" OFF)
if (${MukUtilities_MICCAI2019_OR20})
	add_subdirectory(Conf_MICCAI2019_OR20)
endif()