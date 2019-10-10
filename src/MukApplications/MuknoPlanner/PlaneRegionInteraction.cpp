#include "private/muk.pch"
#include "private/PlaneRegionInteraction.h"
#include "MukCommon/gris_math.h"

#include "AppControllers.h"
#include "VisualizationController.h"
#include "InteractionController.h"
#include "ProblemDefinitionController.h"
#include "PropertyController.h"
#include "VisualizationModel.h"

#include "MukAppModels/AppModels.h"
#include "MukAppModels/VisualizationModel.h"
#include "MukAppModels/ProblemDefinitionModel.h"

#include "MukVisualization/VisCursor3D.h"
#include "MukVisualization/VisPlaneRegion.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/PlaneRegionRepresentation.h"
#include "MukVisualization/PlaneRegionWidget.h"

#include "MukQt/MukQRightClickMenu.h"
#include "MukQt/MedicalMultiViewWidget.h"

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkImplicitPlaneWidget2.h>

namespace
{
  const char* TemporalObjName = "TemporalPlaneRegion";
}

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(PlaneRegionInteraction);

  /**
  */
  PlaneRegionInteraction::PlaneRegionInteraction()
    : IMukInteraction()
  {
  }

  /**
  */
  void PlaneRegionInteraction::finalize() 
  {
    if (mpRegion)
      mpRegion->widget()->SetProcessEvents(0);
    if (mpModels->pVisModel->hasAbstractObject(TemporalObjName))
      mpControls->mpVisControl->deleteAbstractObject(TemporalObjName);
  }

  /** \brief sets up the interpolator and synchronizes the controls
  */
  void PlaneRegionInteraction::initialize(VisPlaneRegion* pObj)
  {
    if (!pObj)
    {
      auto tempObj = std::make_shared<VisPlaneRegion>(TemporalObjName);
      mpControls->mpVisControl->addAbstractObject(tempObj);
      pObj = dynamic_cast<VisPlaneRegion*>(mpModels->pVisModel->getVisScene()->getObject(TemporalObjName).get());
      pObj->setRenderer(mpRenderer);
      auto origin = mpControls->mpVisControl->requestCursor3D()->getPosition(); // origin of the Widget
      double bounds[6] = { origin.x() - 5.0, origin.x() + 5.0, origin.y() - 5.0, origin.y() + 5.0, origin.z() - 5.0, origin.z() + 5.0 };
      pObj->representation()->PlaceWidget(bounds);
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
  void PlaneRegionInteraction::OnRightButtonUp()
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
  void PlaneRegionInteraction::discard()
  {
    finishInteraction();
  }

  /** \brief return to DefaultInteraction3D
  */
  void PlaneRegionInteraction::finishInteraction()
  {
    if (mpRegion)
      mpRegion->widget()->SetProcessEvents(0);
    if (mpModels && mpModels->pVisModel->hasAbstractObject(TemporalObjName))
      mpControls->mpVisControl->deleteAbstractObject(TemporalObjName);
    mpModels->pVisModel->render();
    mpControls->mpInteract->setDefaultInteraction();
  }

  /**
  */
  void PlaneRegionInteraction::accept(EnType type)
  {
    auto pVis = std::make_unique<VisPlaneRegion>("dummy");
    auto pObj = std::dynamic_pointer_cast<VisPlaneRegion>(mpModels->pVisModel->getVisScene()->getObject(TemporalObjName));
    pVis->swap(pObj.get());
    pVis->widget()->SetProcessEvents(0);
    mpRegion = nullptr;
    if (type == enAsStart)
      mpControls->mpProbDefControl->addStartRegion(std::move(pVis));
    else
      mpControls->mpProbDefControl->addGoalRegion(std::move(pVis));
    finishInteraction();
  }
}
}



