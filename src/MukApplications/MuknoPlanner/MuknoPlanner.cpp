#include "private/muk.pch"
#include "MuknoPlanner.h"

#include "private/HandlerTextEditLog.h"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/EvaluationModel.h"
#include "MukAppModels/LocalEnvironment.h"
#include "MukAppModels/VisualizationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/ProblemDefinitionModel.h"
#include "MukAppModels/PropertyModel.h"
#include "MukAppModels/SelectionModel.h"
#include "MukAppModels/WorldVisualizationModel.h"
#include "NavigationThread.h"

#include "AlgorithmController.h"
#include "InteractionController.h"
#include "SelectioNController.h"
#include "ToolbarController.h"

#include "MukQt/MedicalMultiViewWidget.h"
#include "MukQt/MukQMenuBar.h"
#include "MukQt/PropertyWidget.h"

#include "gstd/DynamicPropertyManager.h"

#include <qsize.h>
#include <qevent.h>
#include <qtimer.h>
#include <qdesktopwidget.h>

#include <boost/filesystem.hpp>

namespace gris
{
namespace muk
{
  /**
  */
  MuknoPlanner::MuknoPlanner()
    : mpMasterWindow (std::make_unique<MuknoPlannerMainWindow>())
    , mpModels(std::make_unique<AppModels>())
    , mpControls(std::make_unique<AppControllers>())
  {
  }

  /**
  */
  MuknoPlanner::~MuknoPlanner()
  {
    try
    { // can throw an io exception
      mpModels->pLocal->save();
      // break down any pending interaction
      mpControls->mpInteract->setDefaultInteraction();
    }
    catch (std::exception& e)
    {
      LOG_LINE << "could not save local data: " << e.what();;
    }
  }

  /**
  */
  void MuknoPlanner::init(const std::string& exe)
  {
    // Prepare Executable and Working Dir
    boost::filesystem::path exePath(exe);
    exePath = exePath.parent_path();
    std::string executableDir;
    if (exePath.is_absolute())
      executableDir = exePath.string();
    else
      executableDir = (boost::filesystem::current_path() / exePath).string();
    auto workingDir = boost::filesystem::current_path().string();
    // set the working directory to the application
    if (!executableDir.empty() && executableDir != workingDir)
      boost::filesystem::current_path(executableDir);
    // set executableDir to LocalEnvironment
    mpModels->pLocal->setBinaryDirectory(executableDir);

    // Initialize MainWindow
    mpMasterWindow->setupUi(this);

    // Initialize Log-Window
    mpWrapperLog = std::make_unique<QtLogWrapper>();
    HandlerTextEditLog::Builder logBuilder;
    logBuilder.mpTextEdit = mpMasterWindow->mpTextEditLog;
    logBuilder.mpWrapper = mpWrapperLog.get();
    mpHandlerLog = std::make_unique<HandlerTextEditLog>(logBuilder);

    mpControls->setModels(mpModels.get());
    mpControls->setMainWindow(mpMasterWindow.get());

    // Initialize Models
    mpModels->pAppModel->setExecutableDir(executableDir);
    mpModels->pAppModel->setWorkingDir(workingDir);
    mpModels->pVisModel->setScene(mpModels->pAppModel->getScene());
    mpModels->pVisModel->setVisWindow(mpMasterWindow->mpMedMultiViewWidget);
    mpModels->pProbDefModel->setScene(mpModels->pAppModel->getScene());
    mpModels->pWorldVisModel->setAxialSliceWidget(mpMasterWindow->mpMedMultiViewWidget->getAxialSliceWidget());
    mpModels->pWorldVisModel->setSagittalSliceWidget(mpMasterWindow->mpMedMultiViewWidget->getSagittalSliceWidget());
    mpModels->pWorldVisModel->setCoronalSliceWidget(mpMasterWindow->mpMedMultiViewWidget->getCoronalSliceWidget());
    mpModels->pWorldVisModel->set3DWindow(mpMasterWindow->mpMedMultiViewWidget->get3DWindow());
    mpModels->pWorldVisModel->initialize();
    mpMasterWindow->mpTabContainer->setCurrentIndex(0); // mpTabPlanning -> currentWindow
    mpModels->pLocal->load();
    mpControls->initialize();

    setupConnections();

    // post connection-setup steps
    // read load localEnvironment
    mpModels->pAppModel->getScene()->setLocalBasePath(mpModels->pLocal->getRawDataPath());
    // start Navigation Thread as first action in the main Event Queue
    QTimer::singleShot(0, std::bind(
      &NavigationThread::start,
      mpControls->mpNaviThread.get(),
      QThread::InheritPriority
      ));
    // finished everything 
    mpModels->pVisModel->render();
    LOG_LINE << "MuknoPlanner initialized!";      
  }

  /** \brief work around to fix buggy member function showMaxmized() on windows

    windows is too stupid to show the app maximized at start up
    https://stackoverflow.com/questions/27157312/qt-showmaximized-not-working-in-windows
  */
  void MuknoPlanner::showInStartUpMode()
  {
    showMaximized();
    QRect rec = QApplication::desktop()->screenGeometry();
    auto height = rec.height();
    auto width = rec.width();
    resize(width, height);
  }

  /**
  */
  void MuknoPlanner::writePropertyFile() const
  {
    auto& manager = gstd::GetDynamicPropertyManager();
    manager.initialize("Mukno", "init.xml");
    manager.connect("SurgeryPlanning");
    // process MukScene
    auto pScene    = mpModels->pAppModel->getScene();
    auto pVisScene = mpModels->pVisModel->getVisScene();
    {
      manager.connect("MukScene.Obstacles");
      auto keys = pScene->getObstacleKeys();
      /*for (const auto& key : keys)
      {
        manager.login(*pScene->getObstacle(key), "Obstacle");
        auto pVis = pVisScene->getObstacle(key);
        if (nullptr!=pVis.get())
          manager.login(*pVis, "Obstacle");
      }*/

      manager.connect("MukScene.PathCollections");
      keys = pScene->getPathKeys();
      for (const auto& key : keys)
      {
        // manager.login(pScene->getPathCollection(key), "PathCollection");
        //manager.login(*pVisScene->getPathCollection(key), "PathCollection");
      }
    }

    // algorithms
    manager.connect("SurgeryPlanning");
    {
      manager.login(*pScene->getPlanner(), "Planner");
      manager.login(*pScene->getPruner(), "Pruner");
      manager.login(*pScene->getInterpolator(), "Interpolator");
    }
    {
      manager.login(*(mpModels->pVisModel), "Visualization");
    }
    manager.disconnect();
  }

  /**
  */
  void MuknoPlanner::setupConnections()
  {
    mpControls->setupConnections();
  }

  /**
  */
  void MuknoPlanner::closeEvent(QCloseEvent* e)
  {
    mpControls->mpSelectControl->finalize();
  }

  /**
  */
  std::unique_ptr<MuknoPlanner> MuknoPlanner::create(const std::string& exe)
  {
    auto result = std::unique_ptr<MuknoPlanner>(new MuknoPlanner());
    result->init(exe);
    return result;
  }

  /**
  */
  void MuknoPlanner::handleFileInput(const std::string& filename)
  {
    namespace fs = boost::filesystem;
    const auto fn = fs::path(filename);
    if (!fs::is_regular_file(fn))
      return;

    enum FileTypes
    {
      enScene,
      enImage,
      enAlgorithm,
      enUnknown
    };

    std::vector<std::vector<std::string>> supportedExtensions =
    {
      {".mukscene"},
      {".mhd", ".mha"},
      {".alg"},
    };

    FileTypes enFileType = enUnknown;
    auto ext = fn.extension().string();
    auto cmp = [&] (const std::string& str) { return str == ext; };
    if (std::any_of(supportedExtensions[enScene].begin(), supportedExtensions[enScene].end(), cmp))
      enFileType = enScene;
    else if (std::any_of(supportedExtensions[enImage].begin(), supportedExtensions[enImage].end(), cmp))
      enFileType = enImage;
    else if (std::any_of(supportedExtensions[enAlgorithm].begin(), supportedExtensions[enAlgorithm].end(), cmp))
      enFileType = enAlgorithm;

    switch (enFileType)
    {
      case enScene:
        mpControls->mpToolbar->loadScene(filename);
        break;
      case enImage:
        mpControls->mpToolbar->loadCTFile(filename);
        break;
      case enAlgorithm:
        mpControls->mpAlgorithmController->loadAlgorithm(filename, false);
        break;
      case enUnknown:
        LOG_LINE << "Unsupported file extension: " << fn.extension().string();
    };
  }
}
}
