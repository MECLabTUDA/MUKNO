#include "private/muk.pch"
#include "AppControllers.h"
#include "MenuBarController.h"
#include "ToolBarController.h"
#include "PlanningController.h"

#include "ApplicationModel.h"
#include "PlanningModel.h"
#include "MukAppModels/VisualizationModel.h"
#include "LocalEnvironment.h"

#include "MukCommon/PlannerFactory.h"
#include "MukCommon/PathCollection.h"

#include "MukPathPlanning/QuickPlanningConfigurator.h"

#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisPathCollection.h"

#include "MukQt/MuknoPlannerMainWindow.h"
#include "MukQt/MukQMenuBar.h"
#include "MukQt/QuickPlanningConfigurationWindow.h"
#include "MukQt/TabPlanning.h"

namespace gris
{
namespace muk
{
  /**
  */
  void MenuBarController::setupConnnections()
  {
    auto* pBar = mpMainWindow->mMenuBar;
    connect(pBar, &MukQMenuBar::loadSceneClicked, this, &MenuBarController::loadScene);
    connect(pBar, &MukQMenuBar::setDefaultFocusClicked, this, &MenuBarController::setDefaultFocus);
    connect(pBar, &MukQMenuBar::savePathClicked, this, &MenuBarController::savePath);
    connect(pBar, &MukQMenuBar::loadPathClicked, this, &MenuBarController::loadPath);
    connect(pBar, &MukQMenuBar::quickPlanningConfigRequested, this, &MenuBarController::evaluateQuickPlanningConfiguration);
  }

  /**
  */
  void MenuBarController::initialize()
  {
    mpMainWindow->mMenuBar->setLastSceneFiles(mpModels->pLocal->getLastSceneFiles());
  }

  /**
  */
  void MenuBarController::loadScene(const std::string& filename)
  {
    mpControls->mpToolbar->loadScene(filename);
  }

  /**
  */
  void MenuBarController::setDefaultFocus()
  {
    mpModels->pVisModel->setDefaultFocus();
  }

  /**
  */
  void MenuBarController::savePath(const std::string& filename)
  {
    mpModels->pAppModel->savePath(filename);
  }

  /**
  */
  void MenuBarController::loadPath(const std::string& filename)
  {
    const auto& key = mpModels->pPlanningModel->getActivePathCollection();
    if (key.empty())
      return;

    auto& coll = mpModels->pAppModel->getScene()->getPathCollection(key);
    const size_t N_old      = coll.getPaths().size();
    mpModels->pAppModel->loadPath(filename);
    const size_t N_newPaths = coll.getPaths().size() - N_old;
    for (size_t i(N_old); i<N_old + N_newPaths; ++i)
    {      
      mpModels->pVisModel->addPath(key, i);
      MukPath interpolatedPath;
      mpModels->pPlanningModel->updateInterpolator(i, interpolatedPath);
      mpModels->pVisModel->getVisScene()->getPathCollection(key)->getMukPath(i)->setData( interpolatedPath );
    }
    mpModels->pVisModel->render();
  }

  /**
  */
  void MenuBarController::evaluateQuickPlanningConfiguration()
  {
    std::vector<std::string> keys;
    GetPlannerFactory().getKeys(keys);
    auto pWindow = std::make_unique<QuickPlanningConfigurationWindow>();
    for (const auto& key: keys)
    {
      pWindow->addConfig(QuickPlanningConfigurator::create(key));
    }
    pWindow->reloadConfigs();
    int result = pWindow->exec();
    if (result != QDialog::Accepted)
    {
      return;
    }
    mpControls->mpPlanningController->setPlanner(pWindow->getPlanner());
    mpControls->mpPlanningController->setPruner(pWindow->getPruner());
    mpControls->mpPlanningController->setInterpolator(pWindow->getInterpolator());
    mpControls->mpPlanningController->initializeTabPlanning();
    const auto& key = mpModels->pPlanningModel->getActivePathCollection();
    auto& pColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
    const size_t N = pColl->sizeStart();
    /*for (size_t i(0); i<N; ++i)
    {
      auto* pSphereRegion = dynamic_cast<VisMukStateRegion*>(&pColl->getStartRegion(i));
      if (pSphereRegion)
      {
        pSphereRegion->setMukStateResolution(pWindow->getSphereResolution());
      }
    }*/
    mpControls->mpPlanningController->updateProblemDefinition(key);
    mpModels->pAppModel->getScene()->getPlanner()->setCalculationTime(pWindow->getCalcTime());
    mpModels->pAppModel->getScene()->getPlanner()->setStepSize(pWindow->getStepSize());
  }
}
}
