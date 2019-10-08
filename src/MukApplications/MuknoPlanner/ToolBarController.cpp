#include "private/muk.pch"

#include "AppControllers.h"
#include "InteractionController.h"
#include "PlanningController.h"
#include "ProblemDefinitionController.h"
#include "PropertyController.h"
#include "ToolBarController.h"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/LocalEnvironment.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/ProblemDefinitionModel.h"
#include "MukAppModels/VisualizationModel.h"
#include "MukAppModels/WorldVisualizationModel.h"

#include "MukCommon/MukScene.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/vtk_tools.h"

#include "MukImaging/MukImagingIO.h"

#include "MukPathPlanning/InterpolatorLinear.h"

#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisPathCollection.h"

#include "MukQt/MukQToolBar.h"
#include "MukQt/MukQMenuBar.h"
#include "MukQt/MuknoPlannerMainWindow.h"
#include "MukQt/TabPlanning.h"

#include <vtkMetaImageReader.h>

#include <boost/filesystem.hpp>

namespace gris
{
namespace muk
{
  /**
  */
  void ToolBarController::setupConnnections()
  {
    connect(mpMainWindow->mToolBar, &MukQToolBar::focusClicked, this, &ToolBarController::focusOnDefault);
    connect(mpMainWindow->mToolBar, &MukQToolBar::focusOnWaypointClicked, this, &ToolBarController::focusOnWaypoint);
    connect(mpMainWindow->mToolBar, &MukQToolBar::createNewSceneClicked, this, &ToolBarController::clearScene);
    connect(mpMainWindow->mToolBar, &MukQToolBar::loadSceneClicked, this, &ToolBarController::loadScene);
    connect(mpMainWindow->mToolBar, &MukQToolBar::loadCTFileClicked, this, &ToolBarController::loadCTFile);
    connect(mpMainWindow->mToolBar, &MukQToolBar::loadSegmentationFileClicked, this, &ToolBarController::loadSegmentation);
    connect(mpMainWindow->mToolBar, &MukQToolBar::loadObstacleFileClicked, this, &ToolBarController::loadObstacleFromFile);
    connect(mpMainWindow->mToolBar, &MukQToolBar::saveSceneClicked, this, &ToolBarController::saveScene);
  }

  /**
  */
  void ToolBarController::focusOnDefault()
  {
    mpModels->pVisModel->focus();
  }

  /**
  */
  void ToolBarController::focusOnWaypoint()
  {
    mpModels->pVisModel->focusOnWaypoint();
  }

  /**
  */
  void ToolBarController::loadScene(const std::string& filename)
  {
    mpModels->pProbDefModel->unloadAll();
    mpModels->pAppModel->reset();
    mpModels->pVisModel->reset();
    mpModels->pAppModel->loadScene(filename);
    mpModels->pVisModel->synchronize(*mpModels->pAppModel->getScene());
    mpModels->pVisModel->load(filename);
    auto pathKeys = mpModels->pAppModel->getScene()->getPathKeys();
    // Problem: interpolator is not saved for every path -> use standard linear interpolator
    {
      auto oldName = mpModels->pAppModel->getScene()->getInterpolator()->name();
      mpControls->mpPlanningController->setInterpolator(InterpolatorLinear::s_name());
      for (const auto& key : pathKeys)
      {
        mpControls->mpPlanningController->selectPathCollection(key);
        const auto N = mpModels->pAppModel->getScene()->getPathCollection(key).getPaths().size();
        for (size_t i(0); i<N; ++i)
        {
          MukPath interpolatedPath;
          mpModels->pPlanningModel->updateInterpolator(i, interpolatedPath);
          mpModels->pVisModel->getVisScene()->getPathCollection(key)->getMukPath(i)->setData( interpolatedPath );
        }
      }
      mpModels->pPlanningModel->setInterpolator(oldName);
    }
    if (!pathKeys.empty())
    {
      mpControls->mpPlanningController->changePathCollection(pathKeys.back());
      mpMainWindow->mpTabPlanning->setActivePathIdx(mpModels->pPlanningModel->getActivePathIdx());
    }

    mpControls->mpInteract->initialize();
    mpControls->mpPlanningController->initializeTabPlanning();
    mpControls->mpPropControl->initSceneWidget();
    mpModels->pLocal->setAsLastFilename(filename);
    mpMainWindow->mMenuBar->setLastSceneFiles(mpModels->pLocal->getLastSceneFiles());
    mpModels->pVisModel->render();
  }

  /**
  */
  void ToolBarController::loadCTFile(const std::string& filename)
  {
    if ( ! boost::filesystem::is_regular_file(filename))
      throw MUK_EXCEPTION("file does not exist", filename.c_str());
    auto reader = make_vtk<vtkMetaImageReader>();
    reader->SetFileName(filename.c_str());
    reader->Update();
    mpModels->pWorldVisModel->setCtImage(reader->GetOutput());
    mpModels->pWorldVisModel->render();
    emit this->ctFileLoaded();
    LOG_LINE << "Loaded CT-File " << filename;
  }

  /**
  */
  void ToolBarController::loadSegmentation(const std::string& filename)
  {
    if ( ! boost::filesystem::is_regular_file(filename))
      throw MUK_EXCEPTION("file does not exist", filename.c_str());
    auto reader = make_vtk<vtkMetaImageReader>();
    reader->SetFileName(filename.c_str());
    reader->Update();
    mpModels->pWorldVisModel->setSegmentationImage(reader->GetOutput());
    mpModels->pWorldVisModel->render();
    //mpModels->pCtVisModel->riskStructureDataLoaded();
    LOG_LINE << "Loaded Segmentation-File " << filename;
  }

  /**
  */
  void ToolBarController::clearScene()
  {
    mpControls->mpInteract->setDefaultInteraction();
    mpModels->pProbDefModel->unloadAll();
    mpModels->pAppModel->reset();
    mpModels->pVisModel->reset();
    mpModels->pVisModel->synchronize(*mpModels->pAppModel->getScene());
    mpModels->pVisModel->clearAbstractObjects();
    mpControls->mpPlanningController->initializeTabPlanning();
    mpControls->mpProbDefControl->initProbDefVisualization();
    mpControls->mpInteract->initialize();
    mpControls->mpPropControl->initSceneWidget();
    emit this->sceneCleared();
  }

  /**
  */
  void ToolBarController::loadObstacleFromFile(const std::string& filename)
  {
    mpModels->pAppModel->loadObstacle(filename);
    mpModels->pVisModel->synchronize(*mpModels->pAppModel->getScene());
    mpControls->mpPropControl->initSceneWidget();
  }

  /**
  */
  void ToolBarController::saveScene(const std::string& filename)
  {
    mpModels->pAppModel->saveScene(filename);
    mpModels->pVisModel->save(filename);
    mpModels->pLocal->setAsLastFilename(filename);
    mpMainWindow->mMenuBar->setLastSceneFiles(mpModels->pLocal->getLastSceneFiles());
    LOG_LINE << "saved scene to " << filename;
  }

}
}
