#include "private/muk.pch"
#include "Cursor3DRepresentation.h"

#include "MukCommon/muk_common.h"

#include <vtkObjectFactory.h>
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkLine.h>
#include <vtkProperty.h>
#include <vtkBox.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAbstractPicker.h>
#include <vtkImageData.h>
#include <vtkImageSliceMapper.h>

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(Cursor3DRepresentation);

  //----------------------------------------------------------------------------
  /**
  */
  Cursor3DRepresentation::Line::Line()
  {
    lineData   = make_vtk<vtkPolyData>();
    lineMapper = make_vtk<vtkPolyDataMapper>();
    lineActor  = make_vtk<vtkActor>();
    lineMapper->SetInputData(lineData);
    lineActor->SetMapper(lineMapper);
    lineActor->GetProperty()->SetColor(0, 0, 0.9);
    lineActor->GetProperty()->SetOpacity(0.9);
    lineActor->GetProperty()->SetLineWidth(1);

    auto points = make_vtk<vtkPoints>();
    points->SetNumberOfPoints(2);
    lineData->SetPoints(points);

    auto lines = make_vtk<vtkCellArray>();
    vtkIdType pts[2] = { 0,1 };
    lines->InsertNextCell(2,pts);
    lineData->SetLines(lines);
  }

  //----------------------------------------------------------------------------
  /**
  */
  Cursor3DRepresentation::Cursor3DRepresentation()
    : mpImage(nullptr)
  {
    // The initial state
    InteractionState = Cursor3DRepresentation::enOutside;
    
    mPosition[0] = 0;
    mPosition[1] = 0;
    mPosition[2] = 0;
    
    // Define initial boundary
    double bounds[6];
    bounds[0] = -0.5;
    bounds[1] = 0.5;
    bounds[2] = -0.5;
    bounds[3] = 0.5;
    bounds[4] = -0.5;
    bounds[5] = 0.5;
    
    mBBox = make_vtk<vtkBox>();
    mBBox->SetBounds(bounds);
    PlaceWidget(bounds);
  }

  //----------------------------------------------------------------------------
  /**
  */
  Cursor3DRepresentation::~Cursor3DRepresentation()
  {
  }

  //----------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::StartWidgetInteraction(double e[2])
  {
    // Store the start position
    StartEventPosition[0] = e[0];
    StartEventPosition[1] = e[1];
    StartEventPosition[2] = 0.0;
    // Store the start position
    mLastEventPosition[0] = e[0];
    mLastEventPosition[1] = e[1];
    mLastEventPosition[2] = 0.0;
    ComputeInteractionState(static_cast<int>(e[0]),static_cast<int>(e[1]),0);
  }

  //----------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::WidgetInteraction(double e[2])
  {
    // Convert events to appropriate coordinate systems
    auto* camera = Renderer->GetActiveCamera();
    if ( !camera )
    {
      return;
    }

    // Compute the two points defining the motion vector
    double pos[3];
    auto* inter = Renderer->GetRenderWindow()->GetInteractor();
    inter->GetPicker()->Pick(inter->GetEventPosition()[0], inter->GetEventPosition()[1], 
      0,  // always zero.
      Renderer);
    inter->GetPicker()->GetPickPosition(pos);
    
    // Process the motion
    if ( InteractionState == Cursor3DRepresentation::enMoving )
    {
      Translate(pos,pos);
    }

    // Store the start position
    mLastEventPosition[0] = e[0];
    mLastEventPosition[1] = e[1];
    mLastEventPosition[2] = 0.0;
  }

  //----------------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::Translate(double *p1, double *p2)
  {
    if (!mpImage)
      return;
    double origin[3];
    double spacing[3];
    mpImage->GetOrigin(origin);
    mpImage->GetSpacing(spacing);
    
    for (int i(0); i<3; ++i)
    {
      auto sliceNumber = static_cast<int>(std::round((p2[i] - origin[i]) / spacing[i]));
      mPosition[i]     = origin[i] + sliceNumber*spacing[i];
    }

    /*mPosition[0] = p2[0];
    mPosition[1] = p2[1];
    mPosition[2] = p2[2];*/
    BuildRepresentation();
  }

  //----------------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::PlaceWidget(double bds[6])
  {
    double bounds[6];
    AdjustBounds(bds,bounds,mPosition);
    mBBox->SetBounds(bounds);

    for (int i=0; i<6; i++)
    {
      InitialBounds[i] = bounds[i];
    }

    InitialLength = sqrt((bounds[1]-bounds[0])*(bounds[1]-bounds[0]) +
      (bounds[3]-bounds[2])*(bounds[3]-bounds[2]) +
      (bounds[5]-bounds[4])*(bounds[5]-bounds[4]));

    ValidPick = 1; //since we have set up widget
  }

  //----------------------------------------------------------------------------
  /**
  */
  int Cursor3DRepresentation::ComputeInteractionState(int X, int Y, int modify)
  {
    // Okay, we can process this. Try to pick handles first;
    // if no handles picked, then pick the bounding box.
    if (!Renderer || !Renderer->IsInViewport(X, Y))
    {
      InteractionState = Cursor3DRepresentation::enOutside;
      return InteractionState;
    }
    InteractionState = Cursor3DRepresentation::enMoving;
    return InteractionState;
  }

  //----------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::SetInteractionState(int state)
  {
    InteractionState = state;
    // Depending on state, highlight appropriate parts of representation
    // not necessary here atm
  }

  //----------------------------------------------------------------------
  /**
  */
  double* Cursor3DRepresentation::GetBounds()
  {
    BuildRepresentation();
    return mBBox->GetBounds();
  }

  //----------------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::BuildRepresentation()
  {
    double b[6];
    mBBox->GetBounds(b);
    double p[3];
    // line X
    {
      p[0] = b[0];
      p[1] = mPosition[1];
      p[2] = mPosition[2];
      mLines[0].lineData->GetPoints()->SetPoint(0, p);
      p[0] = b[1];
      p[1] = mPosition[1];
      p[2] = mPosition[2];
      mLines[0].lineData->GetPoints()->SetPoint(1, p);
    }
    // line Y
    {
      p[0] = mPosition[0];
      p[1] = b[2];
      p[2] = mPosition[2];
      mLines[1].lineData->GetPoints()->SetPoint(0, p);
      p[0] = mPosition[0];
      p[1] = b[3];
      p[2] = mPosition[2];
      mLines[1].lineData->GetPoints()->SetPoint(1, p);
    }
    // line Z
    {
      p[0] = mPosition[0];
      p[1] = mPosition[1];
      p[2] = b[4];
      mLines[2].lineData->GetPoints()->SetPoint(0, p);
      p[0] = mPosition[0];
      p[1] = mPosition[1];
      p[2] = b[5];
      mLines[2].lineData->GetPoints()->SetPoint(1, p);
    }
    for (int i(0); i<3; ++i)
      mLines[i].lineData->Modified();
    BuildTime.Modified();
    // Rebuild only if necessary
    if ( GetMTime() > BuildTime ||
       (Renderer && Renderer->GetVTKWindow() &&
       (Renderer->GetVTKWindow()->GetMTime() > BuildTime ||
        Renderer->GetActiveCamera()->GetMTime() > BuildTime)) )
    {
      BuildTime.Modified();
    }
  }

  //----------------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::HandlesOn()
  {
    for (int i=0; i<mLines.size(); ++i)
    {
      mLines[i].lineActor->VisibilityOn();
    }
  }

  //----------------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::HandlesOff()
  {
    for (int i=0; i<mLines.size(); ++i)
    {
      mLines[i].lineActor->VisibilityOff();
    }
  }

  //----------------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::ReleaseGraphicsResources(vtkWindow *w)
  {
    for (int j=0; j<mLines.size(); j++)
    {
      mLines[j].lineActor->ReleaseGraphicsResources(w);
    }
  }

  //----------------------------------------------------------------------------
  /**
  */
  int Cursor3DRepresentation::RenderOpaqueGeometry(vtkViewport *v)
  {
    int count=0;
    BuildRepresentation();
    for (int j=0; j<mLines.size(); j++)
    {
      count += mLines[j].lineActor->RenderOpaqueGeometry(v);
    }
    return count;
  }

  //----------------------------------------------------------------------------
  /**
  */
  int Cursor3DRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport *v)
  {
    int count=0;
    BuildRepresentation();
    for (int j=0; j<mLines.size(); j++)
    {
      count += mLines[j].lineActor->RenderTranslucentPolygonalGeometry(v);
    }
    return count;
  }

  //----------------------------------------------------------------------------
  /**
  */
  int Cursor3DRepresentation::HasTranslucentPolygonalGeometry()
  {
    int result=0;
    BuildRepresentation();
    for (int j=0; j<mLines.size(); j++)
    {
      result |= mLines[j].lineActor->HasTranslucentPolygonalGeometry();
    }
    return result;
  }

  //----------------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::getPosition(double d[3])
  {
    for (int i(0); i < 3; ++i)
      d[i] = mPosition[i];
  }

  //----------------------------------------------------------------------------
  /**
  */
  void Cursor3DRepresentation::setPosition(double* d)
  {
    for (int i(0); i < 3; ++i)
      mPosition[i] = d[i];
  }
}
}