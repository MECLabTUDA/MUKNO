#include "private/muk.pch"
#include "ProblemDefinitionController.h"

#include "AppControllers.h"
#include "InteractionController.h"
#include "PlanningController.h"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/ProblemDefinitionModel.h"
#include "MukAppModels/VisualizationModel.h"

#include "MukQt/MedicalMultiViewWidget.h"
#include "MukQt/muk_qt_tools.h"
#include "MukQt/MuknoPlannerMainWindow.h"
#include "MukQt/TabPlanning.h"

#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisSphereRegion.h"
#include "MukVisualization/SphereRegionRepresentation.h"
#include "MukVisualization/SphereRegionWidget.h"
#include "MukVisualization/VisStateRegion.h"

#include "MukCommon/MukStateRegion.h"

#include <QTreeWidget.h>

//#include <Eigen/Dense>

namespace
{
  using namespace gris::muk;
  QTreeWidgetItem* createItem(int id, const MukState& p);
}

namespace gris
{
namespace muk
{
  const double ProblemDefinitionController::MIN_DISTANCE = 0.25; // minimal distance necessary to be accepted as 'close to picked point'

  /**
  */
  void ProblemDefinitionController::initialize()
  {
  }

  /**
  */
  void ProblemDefinitionController::setupConnections()
  {
    auto* pProbDefWidget = mpMainWindow->mpTabPlanning->mpProbDefWidget;
    connect(pProbDefWidget, &ProblemDefinitionWidget::addStartRegionClicked, this, SELECT<void>::OVERLOAD_OF(&ProblemDefinitionController::addStartRegion));
    connect(pProbDefWidget, &ProblemDefinitionWidget::addGoalRegionClicked, this, SELECT<void>::OVERLOAD_OF(&ProblemDefinitionController::addGoalRegion));
    connect(pProbDefWidget, &ProblemDefinitionWidget::deleteRegionClicked, this, SELECT<void>::OVERLOAD_OF(&ProblemDefinitionController::deleteRegion));

    connect(pProbDefWidget, &ProblemDefinitionWidget::reverseClicked, this, &ProblemDefinitionController::reverseRegions);

    connect(pProbDefWidget->mpTreeWidgetWaypointVis, &QTreeWidget::itemSelectionChanged, this, &ProblemDefinitionController::selectVisRegion);
  }

  /**
  */
  void ProblemDefinitionController::selectPathCollection(const std::string& key)
  {
    mpModels->pProbDefModel->loadPathCollection(key);
    showProblemDefinitionOnly(key);
    initProbDefVisualization();
    mpModels->pVisModel->render();
  }

  /**
  */
  void ProblemDefinitionController::deletePathCollection(const std::string& name)
  {
    if (mpModels->pProbDefModel->getKey()==name)
      mpModels->pProbDefModel->unloadAll();
  }

  /** \brief rebuilds the tree visualization
  */
  void ProblemDefinitionController::initProbDefVisualization()
  {
    auto* mpTreeWidget = mpMainWindow->mpTabPlanning->mpProbDefWidget->mpTreeWidgetWaypointVis;
    QSignalBlocker blocker(mpTreeWidget);
    // always clear everything
    mpTreeWidget->clear();
    // rebuild Tree
    const auto& key = mpModels->pProbDefModel->getKey();
    if (key.empty())
    {
      return;
    }
    auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
    auto* pItem = new QTreeWidgetItem();
    {
      pItem->setText(0, "Start Regions");
      mpTreeWidget->addTopLevelItem(pItem);
      size_t N = pVisColl->sizeStart();
      for (size_t i(0); i<N; ++i)
      {
        auto* item = createItem(i, pVisColl->getStartRegion(i).getCenter());
        pItem->addChild(item);
      }
    }
    pItem = new QTreeWidgetItem();
    {
      pItem->setText(0, "Waypoints");
      mpTreeWidget->addTopLevelItem(pItem);
      /*size_t N = pVisColl->sizeWaypoints();
      for (size_t i(0); i<N; ++i)
      {
        auto* item = createItem(i, pVisColl->getWaypoint(i).getCenter());
        pItem->addChild(item);
      }*/
    }
    pItem = new QTreeWidgetItem();
    {
      pItem->setText(0, "Goal Regions");
      mpTreeWidget->addTopLevelItem(pItem);
      size_t N = pVisColl->sizeGoal();
      for (size_t i(0); i<N; ++i)
      {
        auto* item = createItem(i, pVisColl->getGoalRegion(i).getCenter());
        pItem->addChild(item);
      }
    }
    mpTreeWidget->setCurrentItem(mpTreeWidget->topLevelItem(0));
    mpTreeWidget->expandAll();
  }

  /**
  */
  void ProblemDefinitionController::selectVisRegion()
  {
    auto* pProbDefWidget = mpMainWindow->mpTabPlanning->mpProbDefWidget;
    // check if a valid widgetItem was selected
    const auto regionType = pProbDefWidget->getSelectedType();
    const int  index      = pProbDefWidget->getSelectedIndex();
    if (regionType == MukProblemDefinition::enNone || index == -1)
    {
      mpControls->mpInteract->setDefaultInteraction();
      return;
    }

    const auto& key = mpModels->pProbDefModel->getKey();
    if (key.empty())
    {
      return;
    }
    auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);

    switch (regionType)
    {
      case MukProblemDefinition::enStart:
        loadRegion(pVisColl->getStartRegion(index));
        break;
      case MukProblemDefinition::enGoal:
        loadRegion(pVisColl->getGoalRegion(index));
        break;
    }
    mpModels->pVisModel->render();
  }

  /** \brief Adds the passed region to the ProblemDefinitionWidget

    Used when a state region was manually created in the visualization
  */
  void ProblemDefinitionController::addStartRegion(std::unique_ptr<VisStateRegion> pObj)
  {
    if (mpModels->pProbDefModel->addStartRegion(std::move(pObj)))
    {
      initProbDefVisualization();
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      const auto  N = mpModels->pVisModel->getVisScene()->getPathCollection(key)->sizeStart();
      mpMainWindow->mpTabPlanning->mpProbDefWidget->setSelection(MukProblemDefinition::enStart, N-1);
      mpModels->pVisModel->render();
      selectVisRegion();
    }
  }

  /** \brief Adds the passed region to the ProblemDefinitionWidget

    Used when a state region is automatically created, e.g. via in navigation's replanning.
    ATM: just supported for MukStateRegion/replanning. Creates a dummy visualization to use addStartRegion(VisStateRegion)
  */
  void ProblemDefinitionController::addStartRegion(std::unique_ptr<IStateRegion> pObj)
  {
    auto* pInput = dynamic_cast<MukStateRegion*>(pObj.get());
    if (pInput)
    {
      auto pVis = std::make_unique<VisSphereRegion>("ReplanningStart");
      pVis->setState(pInput->getCenter());
      pVis->representation()->setSpherePhi(180 / 2.0 * pInput->getPhi());
      pVis->representation()->setSphereRadius(pInput->getRadius());
      addStartRegion(std::move(pVis));
      mpMainWindow->mpTabPlanning->mpProbDefWidget->setSelection(MukProblemDefinition::enNone, -1);
      mpControls->mpInteract->setDefaultInteraction();
    }
  }

  /** Adds the passed region to the ProblemDefinitionWidget

    Used when a state region was manually created in the visualization
  */
  void ProblemDefinitionController::addGoalRegion(std::unique_ptr<VisStateRegion> pObj)
  {
    if (mpModels->pProbDefModel->addGoalRegion(std::move(pObj)))
    {
      initProbDefVisualization();
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      const auto  N = mpModels->pVisModel->getVisScene()->getPathCollection(key)->sizeGoal();
      mpMainWindow->mpTabPlanning->mpProbDefWidget->setSelection(MukProblemDefinition::enGoal, N-1);
      mpModels->pVisModel->render();
      selectVisRegion();
    }
  }

  /** Adds the passed region to the ProblemDefinitionWidget

    Used when a state region is automatically created, e.g. in navigation's replanning.
    ATM: just supported for MukStateRegion/replanning. Creates a dummy visualization to use addStartRegion(VisStateRegion)
  */
  void ProblemDefinitionController::addGoalRegion(std::unique_ptr<IStateRegion> pObj)
  {
    auto* pInput = dynamic_cast<MukStateRegion*>(pObj.get());
    if (pInput)
    {
      auto pVis = std::make_unique<VisSphereRegion>("ReplanningGoal");
      auto goal = pInput->getCenter();
      pVis->setState(goal);
      pVis->representation()->setSpherePhi(180 / 2.0 * pInput->getPhi());
      pVis->representation()->setSphereRadius(pInput->getRadius());
      addGoalRegion(std::move(pVis));
      mpMainWindow->mpTabPlanning->mpProbDefWidget->setSelection(MukProblemDefinition::enNone, -1);
      mpControls->mpInteract->setDefaultInteraction();
    }
  }

  /**
  */
  void ProblemDefinitionController::addStartRegion()
  {
    mpControls->mpInteract->setDefaultInteraction();
    if (mpModels->pProbDefModel->addStartRegion())
    {
      initProbDefVisualization();
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      const auto  N   = mpModels->pVisModel->getVisScene()->getPathCollection(key)->sizeStart();
      mpMainWindow->mpTabPlanning->mpProbDefWidget->setSelection(MukProblemDefinition::enStart, N-1);
      mpModels->pVisModel->render();
    }
  }

  /**
  */
  void ProblemDefinitionController::addWaypoint()
  {
  }

  /**
  */
  void ProblemDefinitionController::addGoalRegion()
  {
    mpControls->mpInteract->setDefaultInteraction();
    if (mpModels->pProbDefModel->addGoalRegion())
    {
      initProbDefVisualization();
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      const auto  N   = mpModels->pVisModel->getVisScene()->getPathCollection(key)->sizeGoal();
      mpMainWindow->mpTabPlanning->mpProbDefWidget->setSelection(MukProblemDefinition::enGoal, N-1);
      mpModels->pVisModel->render();
    }
  }

  /**
  */
  void ProblemDefinitionController::deleteRegion()
  {   
    auto* pProbDefWidget = mpMainWindow->mpTabPlanning->mpProbDefWidget;
    // check if a valid widgetItem was selected
    const auto regionType = pProbDefWidget->getSelectedType();
    const int  index      = pProbDefWidget->getSelectedIndex();
    if (regionType == MukProblemDefinition::enNone || index == -1)
    {
      return;
    }
    mpControls->mpInteract->setDefaultInteraction();
    if (mpModels->pProbDefModel->deleteRegion(regionType, index))
    {
      initProbDefVisualization();
      mpModels->pVisModel->render();
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      const auto N =  regionType == MukProblemDefinition::enStart  ?
        mpModels->pVisModel->getVisScene()->getPathCollection(key)->sizeStart()
        :
        mpModels->pVisModel->getVisScene()->getPathCollection(key)->sizeGoal();
      if (N>0)
      {
        mpMainWindow->mpTabPlanning->mpProbDefWidget->setSelection(regionType, std::max(0, index-1));
      }
    }
  }

  /**
  */
  void ProblemDefinitionController::deleteRegion(MukProblemDefinition::EnRegionType type, size_t idx)
  {
    if (type == MukProblemDefinition::enNone)
    {
      return;
    }
    const auto index = static_cast<int>(idx);
    mpControls->mpInteract->setDefaultInteraction();
    if (mpModels->pProbDefModel->deleteRegion(type, index))
    {
      initProbDefVisualization();
      mpModels->pVisModel->render();
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      const auto N =  type == MukProblemDefinition::enStart  ?
        mpModels->pVisModel->getVisScene()->getPathCollection(key)->sizeStart()
        :
        mpModels->pVisModel->getVisScene()->getPathCollection(key)->sizeGoal();
      if (N>0)
      {
        mpMainWindow->mpTabPlanning->mpProbDefWidget->setSelection(type, std::max(0, index-1));
      }
    }
  }
    
  /**
  */
  void ProblemDefinitionController::showProblemDefinitionOnly(const std::string& name)
  {
    auto keys = mpModels->pAppModel->getScene()->getPathKeys();
    auto* pVisScene = mpModels->pVisModel->getVisScene();
    for (const auto& key : keys)
    {
      const bool flag = name == key ? true : false;
      auto pColl = pVisScene->getPathCollection(key);
      auto N = pColl->sizeStart();
      for (size_t i(0); i<N; ++i)
        pColl->getStartRegion(i).setVisibility(flag);
      N = pColl->sizeGoal();
      for (size_t i(0); i<N; ++i)
        pColl->getGoalRegion(i).setVisibility(flag);
    }
  }

  /**
  */
  void ProblemDefinitionController::loadRegion(VisStateRegion& region)
  {
    mpControls->mpInteract->setInteraction(region);
  }

  /** \brief Reverse the order of start and goal regions
      
    This is introduced as a temporary solution to fix the original convention of setting start regions as the goal of an intervention.
  */
  void ProblemDefinitionController::reverseRegions()
  {
    const auto& key = mpModels->pPlanningModel->getActivePathCollection();
    mpControls->mpPlanningController->clearPathCollection(key);
    mpModels->pProbDefModel->reverseRegions();
    initProbDefVisualization();
    mpModels->pVisModel->render();
  }
}
}

// ------------------------------------------------------------------------------------------------------------------

namespace
{
  QTreeWidgetItem* createItem(int id, const MukState& p)
  {
    QTreeWidgetItem* nextItem = new QTreeWidgetItem();
    std::string tmp = (boost::format("W%d") % id).str();
    nextItem->setText(0, QString::fromStdString(tmp));
    tmp = (boost::format("(%0.3f, %0.3f, %0.3f)") % p.coords.x() % p.coords.y() % p.coords.z()).str();
    nextItem->setText(1, QString::fromStdString(tmp));
    tmp = (boost::format("(%0.3f, %0.3f, %0.3f)") % p.tangent.x() % p.tangent.y() % p.tangent.z()).str();
    nextItem->setText(2, QString::fromStdString(tmp));
    return nextItem;
  }
}