#include "private/muk.pch"
#include "private/PropertyDetails.h"
#include "PlanningController.h"

#include "AppControllers.h"
#include "InteractionController.h"
#include "ProblemDefinitionController.h"
#include "PropertyController.h"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/VisualizationModel.h"

#include "MukCommon/MukObstacle.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/PlannerFactory.h"
#include "MukCommon/OptimizerFactory.h"
#include "MukCommon/InterpolatorFactory.h"

#include "MukVisualization/VisMukPathGraph.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisStateRegion.h"

#include "MukQt/muk_qt_tools.h"
#include "MukQt/MuknoPlannerMainWindow.h"
#include "MukQt/TabPlanning.h"
#include "MukQt/VTKWindow.h"

#include <vtkPolyData.h>

#include <qcombobox.h>
#include <qspinbox.h>

namespace gris
{
namespace muk
{
  /**
  */
  void PlanningController::initialize()
  {
    auto* pTab = mpMainWindow->mpTabPlanning;
    {
      std::vector<std::string> keys;
      GetPlannerFactory().getKeys(keys);
      pTab->setPlanners(keys);

      keys.clear();
      GetOptimizerFactory().getKeys(keys);
      pTab->setPruners(keys);

      keys.clear();
      GetInterpolatorFactory().getKeys(keys);
      pTab->setInterpolators(keys);

      pTab->setSamplingText(
        StateSamplerData::toString(StateSamplerData::enBBox),
        StateSamplerData::toString(StateSamplerData::enPointList));
    }
  }

  /**
  */
  void PlanningController::setupConnnections()
  {
    auto* pTab = mpMainWindow->mpTabPlanning;
    connect(pTab, &TabPlanning::pathCollectionChanged,        this, &PlanningController::changePathCollection);
    connect(pTab, &TabPlanning::addPathCollectionClicked,     this, &PlanningController::addPathCollection);
    connect(pTab, &TabPlanning::addReplanningClicked,         this, &PlanningController::addReplanningPathCollection);
    connect(pTab, &TabPlanning::deletePathCollectionClicked,  this, &PlanningController::deletePathCollection);
    connect(pTab, &TabPlanning::clearPathCollectionClicked,   this, &PlanningController::clearPathCollection);
    
    connect(pTab, &TabPlanning::changeSamplingClicked, this, &PlanningController::changeSamplingMethod);
    connect(pTab, &TabPlanning::updateProblemDefinitionClicked,  this, &PlanningController::updateProblemDefinition);

    connect(pTab, &TabPlanning::pathIdxChanged,  this, &PlanningController::changePathIndex);
    connect(pTab, 
        SELECT<const std::string&, size_t>::OVERLOAD_OF(&TabPlanning::createMukPathsClicked), 
        this,
        SELECT<const std::string&, size_t>::OVERLOAD_OF(&PlanningController::createPaths));
    connect(pTab, 
        SELECT<const std::string&, double>::OVERLOAD_OF(&TabPlanning::createMukPathsClicked), 
        this,
        SELECT<const std::string&, double>::OVERLOAD_OF(&PlanningController::createPaths));
    connect(pTab, &TabPlanning::updateMukPathClicked,  this, &PlanningController::updatePath);

    connect(pTab, &TabPlanning::updatePlannerClicked, this, &PlanningController::updatePlanner);
    connect(pTab, &TabPlanning::updatePrunerClicked, this, &PlanningController::updatePruner);
    connect(pTab, &TabPlanning::updateInterpolatorClicked, this, &PlanningController::updateInterpolator);

    connect(pTab, &TabPlanning::plannerChanged, this, &PlanningController::setPlanner);
    connect(pTab, &TabPlanning::prunerChanged, this, &PlanningController::setPruner);
    connect(pTab, &TabPlanning::interpolatorChanged, this, &PlanningController::setInterpolator);
  }

  /**
  */
  void PlanningController::changePathCollection(const std::string& key)
  {    
    selectPathCollection(key);
    mpMainWindow->mpTabPlanning->setPathCollection(key);
    mpMainWindow->mpTabPlanning->setActivePathIdx(mpModels->pPlanningModel->getActivePathIdx());
  }
  
  /**
  */
  void PlanningController::changePathIndex(int index)
  {
    mpModels->pPlanningModel->setActivePathIdx(index);
    mpMainWindow->mpTabPlanning->setActivePathIdx(mpModels->pPlanningModel->getActivePathIdx());
  }

  /**
  */
  void PlanningController::addPathCollection(const std::string& key)
  {
    mpModels->pPlanningModel->insertPathCollection(key);
    mpModels->pVisModel->addPathCollection(key);
    mpControls->mpPropControl->addPathCollection(key);
    {
      auto blocker = QSignalBlocker(mpMainWindow->mpTabPlanning);
      mpMainWindow->mpTabPlanning->insertPathCollection(key);
    }
    selectPathCollection(key);
  }

  /**
  */
  void PlanningController::deletePathCollection(const std::string& name)
  {
    const auto oldName = mpModels->pPlanningModel->getActivePathCollection();
    mpModels->pVisModel->deletePathCollection(name);
    mpModels->pPlanningModel->deletePathCollection(name);
    mpControls->mpProbDefControl->deletePathCollection(name);
    mpControls->mpPropControl->deletePathCollection(name);
    const auto& newName = mpModels->pPlanningModel->getActivePathCollection();
    if (oldName != newName)
      selectPathCollection(newName);
    mpModels->pVisModel->render();
  }

  /**
  */
  void PlanningController::addReplanningPathCollection(const std::string& key, int pathIdx, int stateIdx)
  {
    // then create path collection
    std::string newName;
    try
    {
      newName = mpModels->pPlanningModel->addReplanningCollection(key, pathIdx, stateIdx);
    }
    catch(MukException& e)
    {
      auto pScene = mpModels->pAppModel->getScene();
      const auto& paths = pScene->getPathCollection(key).getPaths();
      if (pathIdx >= paths.size())
      {
        mpMainWindow->mpTabPlanning->setActivePathIdx(-1 + (int)paths.size());
        throw e;
      }
      const auto& states = paths[pathIdx].getStates();
      if (stateIdx >= states.size())
      {
        mpMainWindow->mpTabPlanning->setStateIdx(-1 + (int)states.size());
        throw e;
      }
      throw e;
    }
    mpModels->pVisModel->addPathCollection(newName);
    mpControls->mpPropControl->addPathCollection(newName);
    {
      auto blocker = QSignalBlocker(mpMainWindow->mpTabPlanning);
      mpMainWindow->mpTabPlanning->insertPathCollection(newName);
    }
    selectPathCollection(newName);
  }

  /**
  */
  void PlanningController::clearPathCollection(const std::string& name)
  {
    mpModels->pPlanningModel->clearPaths(name);
    mpModels->pVisModel->clearPaths(name);
    mpModels->pVisModel->render();
    emit this->planningChanged();
  }

  /**
  */
  void PlanningController::selectPathCollection(const std::string& key)
  {
    mpControls->mpInteract->setDefaultInteraction(); // break down pending interaction
    mpModels->pPlanningModel->setActivePathCollection(key);
    if (!key.empty())
    {
      mpModels->pPlanningModel->configurePlanning(key);
    }
    mpControls->mpProbDefControl->selectPathCollection(key);
    mpControls->mpPropControl->initSceneWidget();
    emit this->planningChanged();
  }

  /**
  */
  void PlanningController::updatePath(const std::string& name, int pathIdx)
  {
    mpModels->pPlanningModel->updatePath(name, pathIdx);
    mpModels->pVisModel->updatePath(name, pathIdx);
    emit this->planningChanged();
  }

  /**
  */
  void PlanningController::updatePlanner()
  {
    mpModels->pPlanningModel->updatePlanner();
    const auto& key   = mpModels->pPlanningModel->getActivePathCollection();
    const auto  index = mpModels->pPlanningModel->getActivePathIdx();
    auto pVis = mpModels->pVisModel->getVisScene()->getPathCollection(key)->getMukPath(index);
    pVis->setData(mpModels->pAppModel->getScene()->getPathCollection(key).getPaths()[index]);
    mpModels->pVisModel->render();
    LOG_LINE << "planner updated";
  }

  /**
  */
  void PlanningController::updatePruner()
  {
    mpModels->pPlanningModel->updatePruner();
    const auto& key   = mpModels->pPlanningModel->getActivePathCollection();
    const auto  index = mpModels->pPlanningModel->getActivePathIdx();
    auto pVis = mpModels->pVisModel->getVisScene()->getPathCollection(key)->getMukPath(index);
    pVis->setData(mpModels->pAppModel->getScene()->getPathCollection(key).getPaths()[index]);
    mpModels->pVisModel->render();
    LOG_LINE << "pruner updated";
  }

  /**
  */
  void PlanningController::updateInterpolator()
  {    
    const auto& key   = mpModels->pPlanningModel->getActivePathCollection();
    const auto  index = mpModels->pPlanningModel->getActivePathIdx();

    MukPath interpolatedPath;
    mpModels->pPlanningModel->updateInterpolator(index, interpolatedPath);
    mpModels->pVisModel->getVisScene()->getPathCollection(key)->getMukPath(index)->setData( interpolatedPath );

    mpModels->pVisModel->render();
    LOG_LINE << "PlanningModel: interpolator updated";
  }

  /**
  */
  void PlanningController::setPlanner(const std::string& name)
  {
    mpModels->pPlanningModel->setPlanner(name);
    mpControls->mpPropControl->setPlanner(name);
    mpControls->mpPropControl->reloadProperty();
  }

  /**
  */
  void PlanningController::setPruner(const std::string& name)
  {
    mpModels->pPlanningModel->setPruner(name);
    mpControls->mpPropControl->setPruner(name);
    mpControls->mpPropControl->reloadProperty();
  }

  /**
  */
  void PlanningController::setInterpolator(const std::string& name)
  {
    mpModels->pPlanningModel->setInterpolator(name);
    mpControls->mpPropControl->setInterpolator(name);
    mpControls->mpPropControl->reloadProperty();
  }
  
  /**
  */
  void PlanningController::updateProblemDefinition(const std::string& key)
  {
    mpModels->pPlanningModel->updateProblemDefinition(key);
    mpModels->pVisModel->clearPaths(key);
    LOG_LINE << "updated problem definition of path collection '"<< key << "'.";
    emit this->planningChanged();
  }

  /** \brief Reconfigures the Planning Model and tries to computes #numNewPaths paths

    lets the model compute paths repeatedly 
    adds visualizations of the new paths
    logs how many new paths were asked for and how many were actually created

    \param pathCollection name of the path collection which gets configures
    \param numNewPaths    number of paths that will maximally get created
  */
  void PlanningController::createPaths(const std::string& pathCollection, size_t numNewPaths)
  {
    auto& coll = mpModels->pAppModel->getScene()->getPathCollection(pathCollection);
    const size_t N_old = coll.getPaths().size();
    
    mpModels->pPlanningModel->configurePlanning(pathCollection);
    mpModels->pPlanningModel->createPaths(pathCollection, numNewPaths);
    const size_t N_newPaths = coll.getPaths().size() - N_old;
    for (size_t i(N_old); i<N_old + N_newPaths; ++i)
    {      
      mpModels->pVisModel->addPath(pathCollection, i);

      MukPath interpolatedPath;
      mpModels->pPlanningModel->updateInterpolator(i, interpolatedPath);
      mpModels->pVisModel->getVisScene()->getPathCollection(pathCollection)->getMukPath(i)->setData( interpolatedPath );
    }
    if (N_newPaths==0)
    {
      mpControls->mpPropControl->extractPlanningGraph();
    }
    else
    {
      std::string key = "Roadmap_" + mpModels->pPlanningModel->getActivePathCollection();
      std::shared_ptr<VisMukPathGraph> pObj;
      auto keys = mpModels->pVisModel->getAbstractObjectKeys();
      if (std::any_of(keys.begin(), keys.end(), [&] (const auto& str) {return str == key; }))
      {
        pObj = std::dynamic_pointer_cast<VisMukPathGraph>(mpModels->pVisModel->getVisScene()->getObject(key));
        pObj->setVisibility(false);
      }
    }
    mpModels->pVisModel->render();
    emit this->planningChanged();
  }

  /** \brief Reconfigures the Planning Model and tries to computes as many paths as possible in the available time

    lets the model compute paths for a certain amount of time 
    adds visualizations of the new paths
    logs how many new paths were actually created in the available time

    \param pathCollection name of the path collection which gets configures
    \param availableTime  time available (in seconds) for reapeted calls to solve the motion planning problem
  */
  void PlanningController::createPaths(const std::string& pathCollection, double availableTime)
  {
    auto& coll = mpModels->pAppModel->getScene()->getPathCollection(pathCollection);
    const size_t N_old = coll.getPaths().size();

    mpModels->pPlanningModel->configurePlanning(pathCollection);
    mpModels->pPlanningModel->createPaths(pathCollection, availableTime);
    const size_t N_newPaths = coll.getPaths().size() - N_old;
    for (size_t i(N_old); i<N_old + N_newPaths; ++i)
    {      
      mpModels->pVisModel->addPath(pathCollection, i);
      MukPath interpolatedPath;
      mpModels->pPlanningModel->updateInterpolator(i, interpolatedPath);
      mpModels->pVisModel->getVisScene()->getPathCollection(pathCollection)->getMukPath(i)->setData( interpolatedPath );
    }
    if (N_newPaths==0)
    {
      mpControls->mpPropControl->extractPlanningGraph();
    }
    else
    {
      std::string key = "Roadmap_" + mpModels->pPlanningModel->getActivePathCollection();
      std::shared_ptr<VisMukPathGraph> pObj;
      auto keys = mpModels->pVisModel->getAbstractObjectKeys();
      if (std::any_of(keys.begin(), keys.end(), [&] (const auto& str) {return str == key; }))
      {
        pObj = std::dynamic_pointer_cast<VisMukPathGraph>(mpModels->pVisModel->getVisScene()->getObject(key));
        pObj->setVisibility(false);
      }
    }
    mpModels->pVisModel->render();
    emit this->planningChanged();
  }
  
  /**
  */
  void PlanningController::changeSamplingMethod()
  {
    if (mpModels->pPlanningModel->getActivePathCollection().empty())
    {
      LOG_LINE << "No Problem Definition available. Cannot switch sampling methods.";
      mpMainWindow->mpTabPlanning->setSamplingSelection(1); // always works
      return;
    }
    auto* pScene = mpModels->pAppModel->getScene().get();
    using C = StateSamplerData;
    using T = StateSamplerData::StateSamplerType;
    // get Type
    int idx = mpMainWindow->mpTabPlanning->getSamlingMethod();
    T newType;
    if (idx == 1)
      newType = T::enBBox;
    else
      newType = T::enPointList;
    // create data accordingly
    StateSamplerData newData;
    try
    {
      switch (newType)
      {
        case T::enBBox:
          LOG_LINE << "bbox";
          newData.setData(newType, pScene->getPlanner()->getProblemDefinition()->getBounds());
          break;
        case T::enPointList:
          LOG_LINE << "pointList";
          auto pObj = pScene->getObstacle("FreeSpace"); // throws if not available
          std::vector<Vec3d> v;
          auto data = pObj->getData();
          const auto N = data->GetNumberOfPoints();
          for (vtkIdType i(0); i<N; ++i)
          {
            double d[3];
            data->GetPoint(i,d);
            v.push_back(Vec3d(d));
          }
          newData.setData(newType, pScene->getPlanner()->getProblemDefinition()->getBounds(), v);
          break;
      };
    }
    catch (std::exception& e)
    {
      LOG_LINE << e.what();
      mpMainWindow->mpTabPlanning->setSamplingSelection(1); // always works
      throw;
    }
    // set data
    mpModels->pPlanningModel->setSamplingType(newData);
  }

  /**
  */
  void PlanningController::initializeTabPlanning()
  {
    auto pScene = mpModels->pAppModel->getScene();
    auto* pTab  = mpMainWindow->mpTabPlanning;
    auto keys   = pScene->getPathKeys();
    pTab->setPathCollections(keys);
    pTab->setPlanner      (mpModels->pAppModel->getScene()->getPlanner()->name());
    pTab->setPruner       (mpModels->pAppModel->getScene()->getPruner()->name());
    pTab->setInterpolator (mpModels->pAppModel->getScene()->getInterpolator()->name());

    const auto& key = mpModels->pPlanningModel->getActivePathCollection();
    if (key.empty())
    {
      pTab->setSamplingSelection(0);
    }
    else
    {
      // adjust combobox
      pTab->setPathCollection(key);
      // adjust sampling
      auto pProb = pScene->getPlanner()->getProblemDefinition();
      if (nullptr == pProb.get() || nullptr == pProb->getSamplingData())
        pTab->setSamplingSelection(0);
      else
      {
        auto type = pProb->getSamplingData()->getType();
        if (type == 0)
          pTab->setSamplingSelection(1);
        else
          pTab->setSamplingSelection(2);
      }
    }
  }

  /** \brief set the bounds of the active path collection's problem definition's bounds to default (BBox of all obstacles) 
  */
  void PlanningController::setDefaultBounds()
  {
    mpModels->pPlanningModel->setDefaultBounds(mpModels->pPlanningModel->getActivePathCollection());
  };

  /**
  */
  void PlanningController::minimizeBounds()
  {
    mpModels->pPlanningModel->setMinimumBounds(mpModels->pPlanningModel->getActivePathCollection());
  }

  /**
  */
  void PlanningController::addObjectBounds()
  {
    const auto items = mpMainWindow->mpSceneWidget->selectedItems();
    if (items.empty() || items.size() > 1)
      return;
    
    QTreeWidgetItem* pItem = items[0];
    QTreeWidgetItem* pParent = pItem->parent();
    if (pParent == nullptr)
      return;
    
    const auto type = mpMainWindow->mpSceneWidget->indexOfTopLevelItem(pParent);
    std::string name;
    switch(type)
    {
      case enObstacle:
        name = pItem->text(0).toStdString();
        mpModels->pPlanningModel->addObstacleBounds(mpModels->pPlanningModel->getActivePathCollection(), name);
        break;
      case enAlgorithmOutput:
        /// not yet supported, does it need to be?
        break;
      case enAbstractObject:
        /// not yet supported, does it need to be?
        break;
    }
  }

  /**
  */
  void PlanningController::deleteObstacle(const std::string& name)
  {
    mpModels->pAppModel->deleteObstacle(name);
    mpModels->pVisModel->synchronize(*mpModels->pAppModel->getScene());
  }
}
}
