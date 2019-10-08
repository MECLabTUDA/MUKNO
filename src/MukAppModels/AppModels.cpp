#include "private/muk.pch"
#include "AppModels.h"

#include "AlgorithmModel.h"
#include "ApplicationModel.h"
#include "PlanningModel.h"
#include "PropertyModel.h"
#include "SelectionModel.h"
#include "VisualizationModel.h"
#include "ProblemDefinitionModel.h"
#include "WorldVisualizationModel.h"

#include "LocalEnvironment.h"

namespace gris
{
namespace muk
{
  /**
  */
  AppModels::AppModels()
  {
    // create Models
    pAlgorithmModel = std::make_unique<AlgorithmModel>();
    pAppModel       = std::make_unique<ApplicationModel>();
    pLocal          = std::make_unique<LocalEnvironment>();
    pPlanningModel  = std::make_unique<PlanningModel>(pAppModel->getScene());
    pPropModel      = std::make_unique<PropertyModel>();
    pSelectionModel = std::make_unique<SelectionModel>();
    pVisModel       = std::make_unique<VisualizationModel>();
    pProbDefModel   = std::make_unique<ProblemDefinitionModel>();
    pWorldVisModel  = std::make_unique<WorldVisualizationModel>();

    // initialize models
    pAlgorithmModel->setModels(this);
    pAppModel->setModels(this);
    pPlanningModel->setModels(this);
    pPropModel->setModels(this);
    pSelectionModel->setModels(this);
    pVisModel->setModels(this);
    pProbDefModel->setModels(this);
    pWorldVisModel->setModels(this);

    mpModels.push_back(pAlgorithmModel.get() );
    mpModels.push_back(pAppModel      .get() );
    mpModels.push_back(pLocal         .get() );
    mpModels.push_back(pPlanningModel .get() );
    mpModels.push_back(pPropModel     .get() );
    mpModels.push_back(pSelectionModel.get() );
    mpModels.push_back(pVisModel      .get() );
    mpModels.push_back(pProbDefModel  .get() );
    mpModels.push_back(pWorldVisModel .get() );
  }

  /**
  */
  AppModels::~AppModels()
  {
  }

  /**
  */
  bool AppModels::hasModel(const std::string& name) const
  {
    for(const auto* p : mpModels)
    {
      if (name == p->name())
        return true;
    }
    return false;
  }

  /**
  */
  BaseModel* AppModels::getModel(const std::string& name)
  {
    auto iter = std::find_if(mpModels.begin(), mpModels.end(), [&] (const auto& m) { return name == m->name(); });
    if (iter == mpModels.end())
      return nullptr;
    return *iter;
  }

  /**
  */
  BaseModel* AppModels::getModel(const std::string& name) const
  {
    return const_cast<AppModels*>(this)->getModel(name);
  }
}
}