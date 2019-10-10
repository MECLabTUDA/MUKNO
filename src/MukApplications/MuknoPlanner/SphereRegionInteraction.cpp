#include "private/muk.pch"
#include "private/SphereRegionInteraction.h"
#include "MukCommon/gris_math.h"

#include "AppControllers.h"
#include "VisualizationController.h"
#include "InteractionController.h"
#include "PlanningController.h"
#include "ProblemDefinitionController.h"
#include "PropertyController.h"
#include "VisualizationModel.h"

#include "MukVisualization/VisCursor3D.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisSphereRegion.h"
#include "MukVisualization/SphereRegionRepresentation.h"
#include "MukVisualization/SphereRegionWidget.h"

#include "MukQt/MukQRightClickMenu.h"
#include "MukQt/MedicalMultiViewWidget.h"

namespace
{
  const char* TemporalObjName = "TemporalSphereRegion";
}

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(SphereRegionInteraction);

  /**
  */
  SphereRegionInteraction::SphereRegionInteraction()
    : IMukInteraction()
  {
  }

  /**
  */
  void SphereRegionInteraction::finalize()
  {
    if (mpRegion)
    {
      mpRegion->widget()->SetProcessEvents(0);
    }
  }

  /** \brief sets up the interpolator and synchronizes the controls
  */
  void SphereRegionInteraction::initialize(VisSphereRegion* pObj)
  {
    if (!pObj)
    {
      auto tempObj = std::make_shared<VisSphereRegion>(TemporalObjName);
      mpModels->pVisModel->addAbstractObject(tempObj);
      pObj = dynamic_cast<VisSphereRegion*>(mpModels->pVisModel->getVisScene()->getObject(TemporalObjName).get());
      pObj->setRenderer(mpRenderer);
      auto origin = mpControls->mpVisControl->requestCursor3D()->getPosition(); // origin of the Widget
      pObj->representation()->PlaceWidget(MukState(origin, Vec3d(0,0,1)));
      pObj->representation()->setSphereRadius(mSphereRadius);
    }
    else
    {
      pObj->setRenderer(mpRenderer);
      mpControls->mpPropControl->reloadProperty();
      mpRegion = pObj;
    }
    pObj->widget()->SetEnabled(1);
    pObj->widget()->SetProcessEvents(1);
    mpModels->pVisModel->render();
  }

  /** \brief When you get outside of the widget you can get back to the default3dinteraction on rightclick
  */
  void SphereRegionInteraction::OnRightButtonUp()
  {
    vtkInteractorStyleTrackballCamera::OnRightButtonUp();

    auto pMenu = std::make_unique<MukQRightClickMenu>();
    if ( ! mpRegion)
    {
      pMenu->addHandle("Create as Start Region", [&] () { accept(enAsStart); } );
      pMenu->addHandle("Create as Goal Region",  [&] () { accept(enAsGoal); } );
      pMenu->addHandle("Discard", [&] () { discard(); } );
    }
    else
    {
      pMenu->addHandle("Finish", [&] () { discard(); } );
    }
    pMenu->exec();
  }


  /** \brief return to DefaultInteraction3D
  */
  void SphereRegionInteraction::discard()
  {
    finishInteraction();
  }

  /** \brief return to DefaultInteraction3D
  */
  void SphereRegionInteraction::finishInteraction()
  {
    if (mpRegion)
      mpRegion->widget()->SetProcessEvents(0);
    if (mpModels->pVisModel->hasAbstractObject(TemporalObjName))
      mpControls->mpVisControl->deleteAbstractObject(TemporalObjName);
    
    mpModels->pVisModel->render();
    mpControls->mpInteract->setDefaultInteraction();
  }

  /**
  */
  void SphereRegionInteraction::accept(EnType type)
  {
    auto pVis = std::make_unique<VisSphereRegion>("dummy");
    auto pObj = std::dynamic_pointer_cast<VisSphereRegion>(mpModels->pVisModel->getVisScene()->getObject(TemporalObjName));
    pVis->swap(pObj.get());
    pVis->widget()->SetProcessEvents(0);
    if (type == enAsStart)
      mpControls->mpProbDefControl->addStartRegion(std::move(pVis));
    else
      mpControls->mpProbDefControl->addGoalRegion(std::move(pVis));
    finishInteraction();
  }

  /**
  */
  void SphereRegionInteraction::setSphereRadius(double r)
  {
    mSphereRadius = r;
  }
}
}



