# MUKNO
Preoperative planning framework for temporal bone surgery featuring image processing and trajectory planning 

Overview

This codebase was published along with our IPCAI 2019 submissions

    "Towards an automatic preoperative pipeline for image-guided temporal bone surgery".(IPCAI 2019)
    "Planning for Flexible Surgical Robots via Bézier Spline Translation".(RA-L 2019)
    "Optimizing Clearance of Bézier Spline Trajectories for Minimally-Invasive Surgery".(MICCAI 2019)
    "Preoperative Planning for Guidewires employing Shape-Regularized Segmentation and Optimized Trajectories".(MICCAI 2019 OR 2.0 workshop)

It includes necessary c++ libraries for image processing, trajectory planning and optimization as well as a GUI to interactively setup a surgical motion planning problem.
License

Published under the GNU LGPL license version 3.
Dependencies

    boost (https://www.boost.org/)
    Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page)
    loki (http://loki-lib.sourceforge.net/)
    GrisFramework (https://github.com/MECLabTUDA/GrisFramework)
    ACVD (https://github.com/valette/ACVD)
    The Computational Geometry Algorithms Library (CGAL) (https://www.cgal.org/)
    Open Motion Planning Library (OMPL) (http://ompl.kavrakilab.org/)
    Qt (https://www.qt.io/)
    The Visualization Toolkit (VTK) (https://vtk.org/)
    Insight Segmentaiton and Registration Toolkit (ITK) (https://itk.org/)
    Gurobi (when using the convex optimization option)

Known good configurations

Tested only with Microsoft Visual Studio 2015, x64, release.

    boost 1.59
    Eigen 3.2.7
    loki -
    GrisFramework 0.6
    ACVD -
    CGAL 4.7
    OMPL 1.2.1
    Qt 5.7
    VTK 7.1.1
    ITK 4.11
    Gurobi 8.0.0

Libraries

    MukCommon includes interfaces, basic geometric tools and classes shared by the other libraries
    MukAlgorithm and MukImaging are used to create an image pipeline as a direct acyclic graph and wraps, among others, ITK and VTK algorithms.
    MukPathPlanning defines motion planning algorithms, e.g. a Spline-Based RRT-connect.
    MukEvaluation features a small set of statistic components.
    MukNavigation includes a dummy navigator to simulate insertion of instruments along the paths and and an interface to write own plugins, e.g. based on the PlusToolkit.
    MukVisualization provides classes that visualize trajectories, search graphs, surfaces meshes and other stuff via VTK.
    MukQt provides widget for the GUI using Qt.
    MukAppModels wraps the above to provide short and easy handling of the concepts of running image processing pipelines, perform trajectory planning, visualize and evaluate results.

Executables

using CMake activate the application

    MukUtilities_IPCAI2019 to include the executable for running experiments based on our IPCAI publication. Unfortunately, we cannot make the medical data publically available at the moment, so you have to find other data to perform own experiments. We suggest you try the publically available data set published with this paper: "Gerber, Nicolas, et al. "A multiscale imaging and modelling dataset of the human inner ear." Scientific data 4 (2017): 170132."
    MukUtilities_MICCAI2019 to include the executable for running experiments based on our MICCAI publication. Beside our closed Temporal Bone Data Set, it makes use of the publicly available data set of the SegTHOR challenge.
    MukUtilities_IROS2019 to include the executable for running experiments based on our RA-L publication. Beside our closed Temporal Bone Data Set, it makes use of the publicly available data set of the SegTHOR challenge.
    MukUtilities_MICCAI2019_OR20 to include the executable for running experiments based on our OR 2.0 publication. It uses of the publicly available data set of the SegTHOR and MMWHS challenge.
    MukUtilities_MuknoPlanner to include the main GUI application.

Each application uses an xml file and various other resources in ./resources/MukApplications/<MukUtilities_XXX> to setup the application. Check in src/MukApplications/<MukUtilities_XXX>/program_options.cpp for further documentation of how to setup the experiments. The GUI works without any resources, but we are still working on a tutorial on how to use it.
