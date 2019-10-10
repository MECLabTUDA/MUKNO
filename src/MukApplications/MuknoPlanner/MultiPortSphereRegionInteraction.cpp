#include "private/muk.pch"
#include "private/MultiPortSphereRegionInteraction.h"
#include "MukCommon/gris_math.h"

#include "AppControllers.h"
#include "VisualizationController.h"
#include "InteractionController.h"
#include "PlanningController.h"
#include "ProblemDefinitionController.h"
#include "PropertyController.h"

#include "MukAppModels/ProblemDefinitionModel.h"
#include "MukAppModels/VisualizationModel.h"

#include "MukVisualization/VisCursor3D.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisMultiPortSphereRegion.h"
#include "MukVisualization/MultiPortSphereRegionRepresentation.h"

#include "MukQt/MukQRightClickMenu.h"
#include "MukQt/MedicalMultiViewWidget.h"

#include <vtkImplicitPlaneWidget2.h>

namespace
{
  const char* TemporalObjName = "SphereRegion";
}

namespace gris
{
  namespace muk
  {
    vtkStandardNewMacro(MultiPortSphereRegionInteraction);

    /**
    */
    MultiPortSphereRegionInteraction::MultiPortSphereRegionInteraction()
      : IMukInteraction()
    {
    }

    /** \brief sets up the interpolator and synchronizes the controls
    */
    void MultiPortSphereRegionInteraction::initialize()
    {
      auto pObj = std::make_shared<VisMultiPortSphereRegion>(TemporalObjName);
      pObj->setRenderer(mpRenderer);
      auto origin = mpControls->mpVisControl->requestCursor3D()->getPosition(); // origin of the Widget
      pObj->representation()->centralize(origin);
      pObj->representation()->setSphereRadius(mSphereRadius);
      pObj->representation()->setPathRadius(mPathRadius);
      pObj->representation()->setAngleThreshold(mAngleThreshold);
      pObj->widget()->On();
      mpModels->pVisModel->addAbstractObject(pObj);
      mpControls->mpPropControl->reloadProperty();
      mpModels->pVisModel->render();
    }
    
    /** \brief When you get outside of the widget you can get back to the default3dinteraction on rightclick
     */
    void MultiPortSphereRegionInteraction::OnRightButtonUp()
    {
      vtkInteractorStyleTrackballCamera::OnRightButtonUp();

      auto pMenu = std::make_unique<MukQRightClickMenu>();
      pMenu->addHandle("Discard", std::bind(&MultiPortSphereRegionInteraction::discard, this));
      pMenu->addHandle("Create as Start Region", std::bind(&MultiPortSphereRegionInteraction::acceptAsStart, this));
      pMenu->addHandle("Create as Goal Region",  std::bind(&MultiPortSphereRegionInteraction::acceptAsGoal, this));
      pMenu->exec();
    }

    /** \brief return to DefaultInteraction3D
    */
    void MultiPortSphereRegionInteraction::discard()
    {
      finishInteraction();
    }

    /** \brief return to DefaultInteraction3D
    */
    void MultiPortSphereRegionInteraction::finishInteraction()
    {
      mpModels->pVisModel->deleteAbstractObject(TemporalObjName);
      mpControls->mpPropControl->reloadProperty();
      mpControls->mpProbDefControl->initProbDefVisualization();
      mpModels->pVisModel->render();
      mpControls->mpInteract->setInteraction("default");
    }

    /**
    */
    void MultiPortSphereRegionInteraction::setSphereRadius(double r)
    {
      mSphereRadius = r;
    }

    /**
    */
    void MultiPortSphereRegionInteraction::setPathRadius(double r)
    {
      mPathRadius = r;
    }

    /**
    */
    void MultiPortSphereRegionInteraction::setAngleThreshold(double alpha)
    {
      mAngleThreshold = alpha;
    }

    /**
    */
    void MultiPortSphereRegionInteraction::acceptAsStart()
    {
      auto pVis = std::make_unique<VisMultiPortSphereRegion>("dummy");
      auto pObj = std::dynamic_pointer_cast<VisMultiPortSphereRegion>(mpModels->pVisModel->getVisScene()->getObject(TemporalObjName));
      pVis->swap(pObj.get());
      pVis->widget()->SetProcessEvents(0);
      pVis->representation()->setHandleVisibility(false);
      mpModels->pProbDefModel->addStartRegion(std::move(pVis));
      finishInteraction();
    }

    /**
    */
    void MultiPortSphereRegionInteraction::acceptAsGoal()
    {
      auto pVis = std::make_unique<VisMultiPortSphereRegion>("dummy");
      auto pObj = std::dynamic_pointer_cast<VisMultiPortSphereRegion>(mpModels->pVisModel->getVisScene()->getObject(TemporalObjName));
      pVis->swap(pObj.get());
      pVis->widget()->SetProcessEvents(0);
      pVis->representation()->setHandleVisibility(false);
      mpModels->pProbDefModel->addGoalRegion(std::move(pVis));
      finishInteraction();
    }
  }
}



