#include "private/muk.pch"
#include "private/DefaultInteraction3D.h"
#include "private/MultiPortSphereRegionInteraction.h"
#include "private/PlaneRegionInteraction.h"
#include "private/SphereRegionInteraction.h"
#include "private/SurfaceCutter.h"
#include "private/SurfaceRegionInteraction.h"

#include "AppControllers.h"
#include "InteractionController.h"
#include "ProblemDefinitionController.h"
#include "PropertyController.h"
#include "VisualizationController.h"

#include "MukCommon/PathCollection.h"
#include "MukCommon/vtk_tools.h"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/VisualizationModel.h"

#include "MukVisualization/VisCursor3D.h"
#include "MukVisualization/VisScene.h"

#include "MukVisualization/VisMultiPortSphereRegion.h"
#include "MukVisualization/VisPlaneRegion.h"
#include "MukVisualization/VisSphereRegion.h"
#include "MukVisualization/VisSurfaceRegion.h"

#include "MukQt/MedicalMultiViewWidget.h"
#include "MukQt/MuknoPlannerMainWindow.h"
#include "MukQt/VtkWindow.h"

#include <vtkInteractorStyle.h>

namespace gris
{
namespace muk
{
  /**
  */
  InteractionController::InteractionController()
  {
  }

  /**
  */
  void InteractionController::initialize()
  {
    mpWindow = mpMainWindow->mpMedMultiViewWidget->get3DWindow();
    setInteraction("default");
  }

  /**
  */
  void InteractionController::setupConnections()
  {
  }

  /**
  */
  void InteractionController::setDefaultInteraction()
  {
    setInteraction("default");
  }

  /** \brief mainly create new state regions
  */
  void InteractionController::setInteraction(const std::string& key)
  {
    if (mpInteractorStyle)
      mpInteractorStyle->finalize();

    if (key == "default")
    {
      mpControls->mpPropControl->reloadProperty();
      //mpControls->mpProbDefControl->initProbDefVisualization();
      auto pObj = make_vtk<DefaultInteraction3D>();
      pObj->setModels(mpModels);
      pObj->setControllers(mpControls);
      pObj->setMainWindow(mpMainWindow);
      pObj->setRenderer(mpWindow->getRenderer());
      pObj->initialize();
      mpWindow->setInteractorStyle(pObj);
      mpInteractorStyle = pObj;
    }
    else if (key == "cutOutObstacle")

    {
      auto pObj = make_vtk<SurfaceCutter>();
      pObj->setModels(mpModels);
      pObj->setControllers(mpControls);
      pObj->setMainWindow(mpMainWindow);
      pObj->setRenderer(mpWindow->getRenderer());
      pObj->initialize();
      mpWindow->setInteractorStyle(pObj);
      mpInteractorStyle = pObj;
    }
    else
    {
      // applay constraints of problem definition to interaction
      const auto& pathKey = mpModels->pPlanningModel->getActivePathCollection();
      if (pathKey.empty())
      {
        LOG_LINE << "no path collection loaded";
        return;
      }
      if (key == "SphereRegionInteraction")
      {
        auto pObj = make_vtk<SphereRegionInteraction>();
        setup(*pObj);
        pObj->initialize();
        mpWindow->setInteractorStyle(pObj);
        mpInteractorStyle = pObj;
      }
      else if (key == "SurfaceRegionInteraction")
      {
        auto pObj = make_vtk<SurfaceRegionInteraction>();
        setup(*pObj);
        pObj->initialize();
        mpWindow->setInteractorStyle(pObj);
        mpInteractorStyle = pObj;
      }
      else if (key == "MultiPortSphereRegionInteraction")
      {
        auto pObj = make_vtk<MultiPortSphereRegionInteraction>();
        pObj->setModels(mpModels);
        pObj->setControllers(mpControls);
        pObj->setMainWindow(mpMainWindow);
        pObj->setRenderer(mpWindow->getRenderer());
        auto& coll = mpModels->pAppModel->getScene()->getPathCollection(pathKey);
        const auto rPath = coll.getProblemDefinition()->getRadius();
        const auto rBall = coll.getProblemDefinition()->getGoalThreshold();
        const auto alpha = coll.getProblemDefinition()->getGoalAngleThreshold();
        pObj->setSphereRadius(rBall);
        pObj->setPathRadius(rPath);
        pObj->setAngleThreshold(alpha);
        pObj->initialize();
        mpWindow->setInteractorStyle(pObj);
        mpInteractorStyle = pObj;
      }
      else if (key == "PlaneRegionInteraction")
      {
        auto pObj = make_vtk<PlaneRegionInteraction>();
        pObj->setModels(mpModels);
        pObj->setControllers(mpControls);
        pObj->setMainWindow(mpMainWindow);
        pObj->setRenderer(mpWindow->getRenderer());
        pObj->initialize();
        mpWindow->setInteractorStyle(pObj);
        mpInteractorStyle = pObj;
      }
    }
    mpControls->mpPropControl->reloadProperty();
    mpModels->pVisModel->render();
  }

  /**
  */
  void InteractionController::showClosestPoint()
  {
    auto pObj = make_vtk<DefaultInteraction3D>();
    pObj->setModels(mpModels);
    pObj->setControllers(mpControls);
    pObj->setMainWindow(mpMainWindow);
    pObj->setRenderer(mpWindow->getRenderer());
    pObj->initialize();
    mpWindow->setInteractorStyle(pObj);
    pObj->OnMouseDoubleClick();
  }

  /** \brief Load the interaction for a specific type of state region

    atm no time for a better solution
  */
  void InteractionController::setInteraction(VisStateRegion& region)
  {
    if (mpInteractorStyle)
      mpInteractorStyle->finalize();
    if (auto* pRegion = dynamic_cast<VisSphereRegion*>(&region))
    {
      auto pObj = make_vtk<SphereRegionInteraction>();
      setup(*pObj);
      pObj->initialize(pRegion);
      mpInteractorStyle = pObj;
      mpWindow->setInteractorStyle(pObj);
    }
    else if (auto* pRegion = dynamic_cast<VisMultiPortSphereRegion*>(&region))
    {
    }
    else if (auto* pRegion = dynamic_cast<VisSurfaceRegion*>(&region))
    {
      auto pObj = make_vtk<SurfaceRegionInteraction>();
      setup(*pObj);
      pObj->initialize(pRegion);
      mpWindow->setInteractorStyle(pObj);
      mpInteractorStyle = pObj;
    }
    else if (auto* pRegion = dynamic_cast<VisPlaneRegion*>(&region))
    {
      auto pObj = make_vtk<PlaneRegionInteraction>();
      setup(*pObj);
      pObj->initialize(pRegion);
      mpWindow->setInteractorStyle(pObj);
      mpInteractorStyle = pObj;
    }
    else
    {
      setDefaultInteraction();
    }
    mpControls->mpPropControl->reloadProperty();
    mpModels->pVisModel->render();
  }

  /**
  */
  void InteractionController::setup(SphereRegionInteraction& obj)
  {
    obj.setModels(mpModels);
    obj.setControllers(mpControls);
    obj.setMainWindow(mpMainWindow);
    obj.setRenderer(mpWindow->getRenderer());
    // applay constraints of problem definition to interaction
    const auto& key = mpModels->pPlanningModel->getActivePathCollection();
    if (key.empty())
    {
      LOG_LINE << "no path collection loaded";
      return;
    }
    auto& coll = mpModels->pAppModel->getScene()->getPathCollection(key);
    const auto rPath = coll.getProblemDefinition()->getRadius();
    const auto rBall = coll.getProblemDefinition()->getGoalThreshold();
    obj.setSphereRadius(rBall);
  }

  /**
  */
  void InteractionController::setup(SurfaceRegionInteraction& obj)
  {
    obj.setModels(mpModels);
    obj.setControllers(mpControls);
    obj.setMainWindow(mpMainWindow);
    obj.setRenderer(mpWindow->getRenderer());
  }

  /**
  */
  void InteractionController::setup(MultiPortSphereRegionInteraction& vis)
  {

  }

  /**
  */
  void InteractionController::setup(PlaneRegionInteraction& obj)
  {
    obj.setModels(mpModels);
    obj.setControllers(mpControls);
    obj.setMainWindow(mpMainWindow);
    obj.setRenderer(mpWindow->getRenderer());
  }
}
}