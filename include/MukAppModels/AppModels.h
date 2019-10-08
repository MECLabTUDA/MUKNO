#pragma once
#include "muk_appmodels_api.h"

#include <memory>

namespace gris
{
  namespace muk
  {
    // Models
    class AlgorithmModel;
    class ApplicationModel;
    class BaseModel;
    class LocalEnvironment;
    class PlanningModel;
    class ProblemDefinitionModel;
    class PropertyModel;
    class SelectionModel;    
    class StateRegionManipulator;
    class VisualizationModel;
    class WorldVisualizationModel;
    

    /** \brief Collection of classes that control the application's work flow
    */
    struct MUK_APP_API AppModels
    {
      public:
        AppModels();
        ~AppModels();

      public:
        bool              hasModel(const std::string& name) const;
        BaseModel*        getModel(const std::string& name);
        BaseModel*        getModel(const std::string& name) const;
 
      public:
        std::unique_ptr<AlgorithmModel>           pAlgorithmModel;
        std::unique_ptr<ApplicationModel>         pAppModel;
        std::unique_ptr<LocalEnvironment>         pLocal;
        std::unique_ptr<PlanningModel>            pPlanningModel;
        std::unique_ptr<ProblemDefinitionModel>   pProbDefModel;
        std::unique_ptr<PropertyModel>            pPropModel;
        std::unique_ptr<SelectionModel>           pSelectionModel;
        std::unique_ptr<VisualizationModel>       pVisModel;
        std::unique_ptr<WorldVisualizationModel>  pWorldVisModel;

        std::vector<BaseModel*> mpModels;
    };

  }
}