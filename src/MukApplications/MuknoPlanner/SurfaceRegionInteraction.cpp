#include "private/muk.pch"
#include "private/SurfaceRegionInteraction.h"

#include "AppControllers.h"
#include "InteractionController.h"
#include "ProblemDefinitionController.h"
#include "PropertyController.h"
#include "VisualizationController.h"

#include "MukAppModels/VisualizationModel.h"

#include "MukVisualization/SurfaceRegionWidget.h"
#include "MukVisualization/SurfaceRegionRepresentation.h"
#include "MukVisualization/VisCursor3D.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisSurfaceRegion.h"

#include "MukQt/ProblemDefinitionWidget.h"
#include "MukQt/MuknoPlannerMainWindow.h"
#include "MukQt/MukQRightClickMenu.h"
#include "MukQt/SphereInteraction.h"
#include "MukQt/TabPlanning.h"

#include <vtkActor.h>
#include <vtkBoxRepresentation.h>
#include <vtkCamera.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPropPicker.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

namespace
{
  const char* TemporalObjName = "TemporalSurfaceRegion";
}

namespace gris
{
  namespace muk
  {
    vtkStandardNewMacro(SurfaceRegionInteraction);

    /**
    */
    SurfaceRegionInteraction::SurfaceRegionInteraction()
      : IMukInteraction()
    {
    }
    
    /**
    */
    void SurfaceRegionInteraction::finalize()
    {
      if (mpRegion)
      {
        mpRegion->widget()->SetProcessEvents(0);
        mpRegion->widget()->Render();
        mpRenderer->GetRenderWindow()->Modified();
      }
      if (mpModels->pVisModel->hasAbstractObject(TemporalObjName))
        mpControls->mpVisControl->deleteAbstractObject(TemporalObjName);
    }

    /** \brief sets up the interpolator and synchronizes the controls

      old region data in the manipulator should not be modified, so unload the index
      create a temporal dummy that could be added as start or goal region
      connect the cursor to the DirectionWidget to manipulater the atm uniform direction of the region
    */
    void SurfaceRegionInteraction::initialize(VisSurfaceRegion* pObj)
    {
      if (!pObj)
      {
        auto tempObj = std::make_shared<VisSurfaceRegion>(TemporalObjName);
        mpControls->mpVisControl->addAbstractObject(tempObj);
        pObj = dynamic_cast<VisSurfaceRegion*>(mpModels->pVisModel->getVisScene()->getObject(TemporalObjName).get());
        pObj->setRenderer(mpRenderer);
        auto directionAncor = Vec3d(mpRenderer->GetActiveCamera()->GetFocalPoint());
        pObj->representation()->PlaceWidget(directionAncor);
      }
      else
      {
        pObj->setRenderer(mpRenderer);
        pObj->representation()->PlaceWidget(pObj->representation()->getDirectionAncor());
        mpControls->mpPropControl->reloadProperty();
      }
      mpRegion = pObj;
      pObj->representation()->setCache(this);
      pObj->widget()->SetEnabled(1);
      pObj->widget()->SetProcessEvents(1);
      pObj->widget()->Render();
      mpModels->pVisModel->render();
    }
    
    /**
    */
    void SurfaceRegionInteraction::OnRightButtonUp()
    {
      vtkInteractorStyleTrackballCamera::OnRightButtonUp();

      auto pMenu = std::make_unique<MukQRightClickMenu>();
      if ( mpModels->pVisModel->hasAbstractObject(TemporalObjName) )
      {
        pMenu->addHandle("Create Start Region", [&] () { accept(enAsStart); });
        pMenu->addHandle("Create Goal Region",  [&] () { accept(enAsGoal); });
        pMenu->addHandle("Discard", [&] () { discard(); });
      }
      else
      {
        pMenu->addHandle("Finish", [&] () { discard(); });
      }
      pMenu->exec();
    }
    
    /**
    */
    void SurfaceRegionInteraction::accept(EnType type)
    {
      auto pVis = std::make_unique<VisSurfaceRegion>("dummy");
      auto pObj = std::dynamic_pointer_cast<VisSurfaceRegion>(mpModels->pVisModel->getVisScene()->getObject(TemporalObjName));
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
    void SurfaceRegionInteraction::discard()
    {
      finishInteraction();
    }

    /**
    */
    void SurfaceRegionInteraction::finishInteraction()
    {
      mpModels->pVisModel->render();
      mpControls->mpInteract->setDefaultInteraction();
    }
}
}