#include "private/muk.pch"
#include "AppControllers.h"
#include "MukAppModels/AppModels.h"

#include "InteractionController.h"
#include "MenuBarController.h"
#include "NavigationThread.h"
#include "PlanningController.h"
#include "ProblemDefinitionController.h"
#include "PropertyController.h"
#include "SelectionController.h"
#include "ToolBarController.h"
#include "VisualizationController.h"
#include "AlgorithmController.h"

#include "MukQt/MuknoPlannerMainWindow.h"

namespace gris
{
namespace muk
{
  /**
  */
  AppControllers::AppControllers()
    : mpToolbar           (std::make_unique<ToolBarController>())
    , mpPlanningController(std::make_unique<PlanningController>())
    , mpPropControl       (std::make_unique<PropertyController>())
    , mpMenuBarController (std::make_unique<MenuBarController>())
    , mpInteract          (std::make_unique<InteractionController>())
    , mpVisControl        (std::make_unique<VisualizationController>())
    , mpNaviThread        (std::make_unique<NavigationThread>())
    , mpSelectControl     (std::make_unique<SelectionController>())
    , mpAlgorithmController(std::make_unique<AlgorithmController>())
    , mpProbDefControl    (std::make_unique<ProblemDefinitionController>())
  {
    mpToolbar->setControllers(this);
    mpPlanningController->setControllers(this);
    mpPropControl->setControllers(this);
    mpMenuBarController->setControllers(this);
    mpInteract->setControllers(this);
    mpVisControl->setControllers(this);
    mpNaviThread->setControllers(this);
    mpSelectControl->setControllers(this);
    mpAlgorithmController->setControllers(this);
    mpProbDefControl->setControllers(this);
  }

  /**
  */
  AppControllers::~AppControllers()
  {
  }

  /**
  */
  void AppControllers::setModels(AppModels* pModels)
  {
    mpToolbar->setModels(pModels);
    mpPlanningController->setModels(pModels);
    mpPropControl->setModels(pModels);
    mpMenuBarController->setModels(pModels);
    mpInteract->setModels(pModels);
    mpVisControl->setModels(pModels);
    mpNaviThread->setModels(pModels);
    mpSelectControl->setModels(pModels);
    mpAlgorithmController->setModels(pModels);
    mpProbDefControl->setModels(pModels);
  }

  /**
  */
  void AppControllers::setMainWindow(MuknoPlannerMainWindow* pWindow)
  {
    mpToolbar->setMainWindow(pWindow);
    mpPlanningController->setMainWindow(pWindow);
    mpPropControl->setMainWindow(pWindow);
    mpMenuBarController->setMainWindow(pWindow);
    mpInteract->setMainWindow(pWindow);
    mpVisControl->setMainWindow(pWindow);
    mpNaviThread->setMainWindow(pWindow);
    mpSelectControl->setMainWindow(pWindow);
    mpAlgorithmController->setMainWindow(pWindow);
    mpProbDefControl->setMainWindow(pWindow);
  }

  /**
  */
  void AppControllers::setupConnections()
  {
    mpToolbar->setupConnnections();
    mpPlanningController->setupConnnections();
    mpPropControl->setupConnections();
    mpMenuBarController->setupConnnections();
    mpInteract->setupConnections();
    mpVisControl->setupConnections();
    mpNaviThread->setupConnections();
    mpSelectControl->setupConnections();
    mpAlgorithmController->setupConnections();
    mpProbDefControl->setupConnections();
  }

  /**
  */
  void AppControllers::initialize()
  {
    mpToolbar->initialize();
    mpPlanningController->initialize();
    mpPropControl->initialize();
    mpMenuBarController->initialize();
    mpVisControl->initialize();
    mpInteract->initialize();
    mpNaviThread->initialize();
    mpSelectControl->initialize();
    mpAlgorithmController->initialize();
    mpProbDefControl->initialize();
  }
}
}