#pragma once

#include <memory>

namespace gris
{
  namespace muk
  {
    struct AppModels;
    class MuknoPlannerMainWindow;

    class AlgorithmController;
    class InteractionController;
    class MenuBarController;
    class NavigationThread;
    class PlanningController;
    class ProblemDefinitionController;
    class PropertyController;
    class SelectionController;
    class ToolBarController;
    class VisualizationController;


    /**
    */
    struct AppControllers
    {
      AppControllers();
      ~AppControllers();

      void setModels(AppModels* pModels);
      void setMainWindow(MuknoPlannerMainWindow* pWindow);
      void setupConnections();
      void initialize();

      std::unique_ptr<AlgorithmController>	 mpAlgorithmController;
      std::unique_ptr<InteractionController> mpInteract;
      std::unique_ptr<MenuBarController>     mpMenuBarController;
      std::unique_ptr<NavigationThread>      mpNaviThread;
      std::unique_ptr<PlanningController>    mpPlanningController;
      std::unique_ptr<ProblemDefinitionController> mpProbDefControl;
      std::unique_ptr<PropertyController>    mpPropControl;
      std::unique_ptr<SelectionController>   mpSelectControl;
      std::unique_ptr<ToolBarController>     mpToolbar;
      std::unique_ptr<VisualizationController> mpVisControl;
    };
  }
}