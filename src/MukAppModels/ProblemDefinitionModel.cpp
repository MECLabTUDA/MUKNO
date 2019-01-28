#include "private/muk.pch"
#include "ProblemDefinitionModel.h"
#include "PlanningModel.h"
#include "VisualizationModel.h"

#include "MukCommon\MukException.h"
#include "MukCommon\Waypoints.h"
#include "MukCommon\PathCollection.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisPlaneRegion.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisSphereRegion.h"
#include "MukVisualization/VisStateRegion.h"
#include "MukVisualization/VisSurfaceRegion.h"
#include "MukVisualization/VisWaypoints.h"

#include "MukQt/MedicalMultiViewWidget.h"
#include "MukQt/VtkWindow.h"

namespace
{
  const int     Invalid_Index = -1;
}

namespace gris
{
namespace muk
{
  /**
  */
  ProblemDefinitionModel::ProblemDefinitionModel()
    : BaseModel()
    , mIndex(Invalid_Index)
    , mKey("")
    , mpVisColl(nullptr)
  {
  }

  /**
  */
  void ProblemDefinitionModel::unloadAll()
  {
    mKey.clear();
    mpVisColl   = nullptr;
  }

  /**
  */
  void ProblemDefinitionModel::loadPathCollection(const std::string& key)
  {
    unloadAll();
    if ( ! key.empty())
    {
      mpVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key).get();
      mKey = key;
    }
  }

  /** \brief adds the specified start region to the problem definition
  */
  bool ProblemDefinitionModel::addStartRegion()
  {
    if (mKey.empty())
    {
      LOG_LINE << "no path collection selected";
      return false;
    }
    const auto& N = mpVisColl->sizeStart();
    const std::string name = "StartRegion" + std::to_string(N);
    auto pVis = std::make_shared<VisSphereRegion>(name);
    pVis->setRenderer(mpModels->pVisModel->getVisWindow()->get3DWindow()->getRenderer());
    mpVisColl->addStartRegion(pVis);
    return true;
  }

  /** \brief adds the specified start region to the problem definition
  */
  bool ProblemDefinitionModel::addStartRegion(std::unique_ptr<VisStateRegion> pObj)
  {
    if (mKey.empty())
    {
      LOG_LINE << "no path collection selected";
      return false;
    }
    const auto& N = mpVisColl->sizeStart();
    const std::string name = "StartRegion" + std::to_string(N);
    pObj->setName(name);
    pObj->setRenderer(mpModels->pVisModel->getVisWindow()->get3DWindow()->getRenderer());
    auto pVis = std::shared_ptr<VisStateRegion>(pObj.release());
    mpVisColl->addStartRegion(pVis);
    return true;
  }

  /** \brief adds the specified goal region to the problem definition
  */
  bool ProblemDefinitionModel::addGoalRegion()
  {
    if (mKey.empty())
    {
      LOG_LINE << "no path collection selected";
      return false;
    }
    const auto& N = mpVisColl->sizeGoal();
    const std::string name = "GoalRegion" + std::to_string(N);
    auto pVis = std::make_shared<VisSphereRegion>(name);
    pVis->setRenderer(mpModels->pVisModel->getVisWindow()->get3DWindow()->getRenderer());
    mpVisColl->addGoalRegion(pVis);
    return true;
  }

  /** \brief adds the specified goal region to the problem definition
  */
  bool ProblemDefinitionModel::addGoalRegion(std::unique_ptr<VisStateRegion> pObj)
  {
    if (mKey.empty())
    {
      LOG_LINE << "no path collection selected";
      return false;
    }
    const auto& N = mpVisColl->sizeGoal();
    const std::string name = "GoalRegion" + std::to_string(N);
    pObj->setName(name);
    pObj->setRenderer(mpModels->pVisModel->getVisWindow()->get3DWindow()->getRenderer());
    auto pVis = std::shared_ptr<VisStateRegion>(pObj.release());
    mpVisColl->addGoalRegion(pVis);
    return true;
  }

  /**
  */
  bool ProblemDefinitionModel::deleteRegion(MukProblemDefinition::EnRegionType en, int index)
  {
    if (mKey.empty())
    {
      LOG_LINE << "no path collection selected";
      return false;
    }
    switch (en)
    {
      case MukProblemDefinition::enStart:
        mpVisColl->deleteStartRegion(index);
        break;
      case MukProblemDefinition::enWaypoint:
        //mpVisColl->deleteWaypoint(index);
        break;
      case MukProblemDefinition::enGoal:
        mpVisColl->deleteGoalRegion(index);
        break;
      default:
        return false;
    }
    return true;
  }

  /** \brief Switches Start and Goal Regions

    Also adjusts the directions.
    would be nice to have exception safety, but this is intended only as a short fix
  */
  void ProblemDefinitionModel::reverseRegions()
  {
    if (mKey.empty())
    {
      LOG_LINE << "no path collection selected";
      return;
    }
    {
      auto pNewColl = std::make_shared<VisPathCollection>(mKey);
      for(const auto& reg : mpVisColl->getGoalRegions())
      {
        pNewColl->addStartRegion(reg);
      }
      for(const auto& reg : mpVisColl->getStartRegions())
      {
        pNewColl->addGoalRegion(reg);
      }
      mpModels->pVisModel->getVisScene()->setPathCollection(pNewColl);
      mpVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(mKey).get();
    }
  }
}
}