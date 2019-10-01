#include "private/muk.pch"
#include "private/DirectionInteractionStyle.h"
#include "SphereInteraction.h"

#include <vtkObjectFactory.h>

#include <vtkIntersectionPolyDataFilter.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkCamera.h>
#include <vtkPoints.h>

#include <vtkCamera.h>
#include <vtkCoordinate.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCoordinate.h>
#include <vtkLineSource.h>


namespace gris
{
  namespace muk
  {
    vtkStandardNewMacro(DirectionInteractionStyle);

    /**
    */
    DirectionInteractionStyle::DirectionInteractionStyle()
      : mousePressed (false)
    {
      
    }

    void DirectionInteractionStyle::OnLeftButtonDown()
    {
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
      mousePressed = true;
    }

    void DirectionInteractionStyle::OnLeftButtonUp()
    {
      vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
      mousePressed =  false;
    }
    
    /**
    */
    void DirectionInteractionStyle::OnMouseMove()
    {
      vtkInteractorStyleTrackballCamera::OnMouseMove();
      if (mousePressed)
      {
        compute();
      }
    }

    /**
    */
    void DirectionInteractionStyle::compute()
    {
      double position[3];
      this->GetCurrentRenderer()->GetActiveCamera()->GetPosition(position);
      Vec3d work(position);
      work.normalize();
      mpInteractor->setDirection(work);
    }

  }
}