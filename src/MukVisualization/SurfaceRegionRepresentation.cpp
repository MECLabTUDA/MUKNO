#include "private/muk.pch"
#include "SurfaceRegionRepresentation.h"

#include "private/SurfaceRegionRepresentationCallback.h"

#include "MukCommon/geometry.h"
#include "MukCommon/muk_common.h"
#include "MukCommon/polygon_3d_tools.h"
#include "MukCommon/vtk_tools.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/PolyDataHandler.h"

#include <vtkActor.h>
#include <vtkAssemblyPath.h>
#include <vtkBoundingBox.h>
#include <vtkCamera.h>
#include <vtkDoubleArray.h>
#include <vtkCoordinate.h>
#include <vtkInteractorObserver.h>
#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPropPicker.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTexturedButtonRepresentation.h>
#include <vtkTransform.h>
#include <vtkImageData.h>
#include <vtkIntersectionPolyDataFilter.h>

namespace
{
  static void CreateImage(vtkSmartPointer<vtkImageData> image, unsigned char *color1, unsigned char *color2);
}

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(SurfaceRegionRepresentation);

  /**
  */
  SurfaceRegionRepresentation::SphereContainer::SphereContainer()
  {
    sphere = make_vtk<vtkSphereSource>();
    mapper = make_vtk<vtkPolyDataMapper>();
    actor  = make_vtk<vtkActor>();
    propActive   = make_vtk<vtkProperty>();
    propInactive = make_vtk<vtkProperty>();
    sphere->SetCenter(2,0,0);
    mapper->SetInputConnection(sphere->GetOutputPort());
    mapper->Modified();
    actor->SetMapper(mapper);
  }

  /**
  */
  SurfaceRegionRepresentation::LineContainer::LineContainer()
  {
    line   = make_vtk<vtkLineSource>();
    mapper = make_vtk<vtkPolyDataMapper>();
    actor  = make_vtk<vtkActor>();
    propActive   = make_vtk<vtkProperty>();
    propInactive = make_vtk<vtkProperty>();
    line->SetPoint1(2,0,0);
    line->SetPoint2(0,0,0);
    mapper->SetInputConnection(line->GetOutputPort());
    actor->SetMapper(mapper);
  }

  /**
  */
  SurfaceRegionRepresentation::PolygonContainer::PolygonContainer()
  {
    buttonRep = make_vtk<vtkTexturedButtonRepresentation2D>();
    buttonRep->SetNumberOfStates(2);
    buttonWidget = make_vtk<vtkButtonWidget>();
    callback = make_vtk<SurfaceRegionRepresentationCallback>();
    selectedPoints = make_vtk<vtkPolyData>();
    selectedPointsMapper = make_vtk<vtkPolyDataMapper>();
    selectedPointsActor = make_vtk<vtkActor>();
    selectedPointsActor->SetMapper(selectedPointsMapper);
    selectedPointsMapper->SetInputData(selectedPoints);
  }
  /**
  */
  SurfaceRegionRepresentation::ArrowContainer::ArrowContainer()
  {
    propActive   = make_vtk<vtkProperty>();
    propInactive = make_vtk<vtkProperty>();
  }

  /** \brief Default Initialisation of the Representation
  */
  SurfaceRegionRepresentation::SurfaceRegionRepresentation()
    : mHandlePicker     (make_vtk<vtkCellPicker>())
    , mOutlineProperty  (make_vtk<vtkProperty>())
    , mBoundingBox      (make_vtk<vtkBox>())
    , mLastPicker       (nullptr)
    , mCurrentHandle    (nullptr)
    , mScaleRation(0.1)
  {
    // The initial state
    InteractionState = enOutside;
    // Handle size is in pixels for this widget
    HandleSize = 5.0;
    // Set up the initial properties
    CreateDefaultProperties();
    // Place Widget
    double bounds[6] = { -0.5, 0.5, -0.5, 0.5, -0.5, 0.5 };
    PlaceWidget(bounds);
    // Manage Picking
    mHandlePicker->SetTolerance(0.001);
    mHandlePicker->AddPickList(mSphere.actor);
    mHandlePicker->PickFromListOn();
    // Create two images for texture of the buttonwidget
    auto image1 = make_vtk<vtkImageData>();
    auto image2 = make_vtk<vtkImageData>();
    unsigned char colorBanana[3] = { 227, 207, 87 };
    unsigned char colorTomato[3] = { 255, 99, 71 };
    CreateImage(image1, colorTomato, colorTomato);
    CreateImage(image2, colorBanana, colorBanana);
    mPoly.buttonRep->SetButtonTexture(0, image1);
    mPoly.buttonRep->SetButtonTexture(1, image2);
    // setup polygon interaction
    mPoly.buttonRep->SetPlaceFactor(1);
    mPoly.callback->mButton     = mPoly.buttonWidget;
    mPoly.callback->mSetPointsCallback = [&] (vtkSmartPointer<vtkPoints> pObj) { setSelectedPoints(pObj); };
    mPoly.buttonWidget->SetRepresentation(mPoly.buttonRep);
    mPoly.observerTag = mPoly.buttonWidget->AddObserver(vtkCommand::StateChangedEvent, mPoly.callback);
    // a line is only shown when there are selected points
    mLine.actor->SetVisibility(false);
  }

  /**
  */
  void SurfaceRegionRepresentation::setDirectionAncor(const Vec3d& p)
  {
    mSphere.sphere->SetCenter(p.x(), p.y(), p.z());
    mLine.line->SetPoint1(p.x(), p.y(), p.z());
    BuildRepresentation();
  }

  /**
  */
  void SurfaceRegionRepresentation::setSurfacePoints(const std::vector<Vec3d>& input)
  {
    auto points = make_vtk<vtkPoints>();
    for (const auto& p : input)
    {
      points->InsertNextPoint(p.x(), p.y(), p.z());
    }
    const auto cache = mLine.actor->GetVisibility();
    setSelectedPoints(points); // this might change the visibility of the line actor
    if (cache != mLine.actor->GetVisibility())
      mLine.actor->SetVisibility(cache);
    BuildRepresentation();
  }

  /**
  */
  void SurfaceRegionRepresentation::setSelectedPoints(vtkSmartPointer<vtkPoints> points)
  {
    mPoly.selectedPoints->Initialize();
    mPoly.selectedPoints->SetPoints(points);
    PolyDataHandler::addVertices(mPoly.selectedPoints);
    mPoly.selectedPointsMapper->Modified();
    const auto N = points->GetNumberOfPoints();
    mLine.actor->SetVisibility( N > 0 );
    if (N != 0)
    {
      Vec3d centroid;
      for (vtkIdType i(0); i<N; ++i)
      {
        double d[3];
        mPoly.selectedPoints->GetPoint(i,d);
        for (int j(0); j<3; ++j)
          centroid[j] += d[j];
      }
      centroid /= N;
      mLine.line->SetPoint2(centroid.data());
      mLine.line->Modified();
    }
    Modified();
    BuildRepresentation();
  }

  /**
  */
  void SurfaceRegionRepresentation::SetRenderer(vtkRenderer *ren)
  {
    vtkWidgetRepresentation::SetRenderer(ren);
    mPoly.callback->mInteractor = Renderer->GetRenderWindow()->GetInteractor();
    mPoly.callback->mRenderer = Renderer;
    mPoly.buttonWidget->SetInteractor(Renderer->GetRenderWindow()->GetInteractor());
    BuildRepresentation();
  }

  /**
  */
  void SurfaceRegionRepresentation::StartWidgetInteraction(double e[2])
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

  /**
  */
  void SurfaceRegionRepresentation::placeButtonWidget()
  {
    if ( ! Renderer )
      return;
    // Place Button Widget
    auto upperRight = make_vtk<vtkCoordinate>();
    upperRight->SetCoordinateSystemToNormalizedDisplay();
    upperRight->SetValue(1.0, 1.0);
    double bds[6];
    double sz = 50.0;
    bds[0] = upperRight->GetComputedDisplayValue(Renderer)[0] - sz;
    bds[1] = bds[0] + sz;
    bds[2] = upperRight->GetComputedDisplayValue(Renderer)[1] - sz;
    bds[3] = bds[2] + sz;
    bds[4] = bds[5] = 0.0;
    mPoly.buttonRep->PlaceWidget(bds);
  }

  /** \brief Rebuilds only if necessary
  */
  void SurfaceRegionRepresentation::BuildRepresentation()
  {
    if ( ! Renderer )
      return;
    // repaint the button widget if render window changes
    if (Renderer->GetVTKWindow()->GetMTime() > BuildTime)
    {
      placeButtonWidget();
    }
    // change size handles if camera or render window changes
    if (Renderer->GetVTKWindow()->GetMTime() > BuildTime ||
      Renderer->GetActiveCamera()->GetMTime() > BuildTime)
    {
      SizeHandles();
    }
    // build the poly data if it was changed
    if ( GetMTime() > BuildTime)
    {
      // Place Line
      mLine.line->SetPoint1(mSphere.sphere->GetCenter());
      if (mPoly.selectedPoints->GetNumberOfPoints() == 0)
      {
        mLine.line->SetPoint2(mSphere.sphere->GetCenter());
      }
      placeArrows();
      BuildTime.Modified();
    }
  }

  //----------------------------------------------------------------------
  /**
  */
  void SurfaceRegionRepresentation::placeArrows()
  {
    mArrows.arrows.clear();
    const auto ancor = getDirectionAncor();
    std::vector<MukState> result;
    const auto N = mPoly.selectedPoints->GetNumberOfPoints();
    for (vtkIdType i(0); i<N; ++i)
    {
      MukState next;
      mPoly.selectedPoints->GetPoint(i, next.coords.data());
      // TODO_: -1 is a hack, because controllers / models cannot distinguish properly betweeen start and goal region
      next.tangent = (-1)*(ancor - next.coords).normalized();
      result.push_back(next);
    }
    for (const auto& state : result)
    {
      mArrows.arrows.addArrow(Arrow::create(state.coords, state.tangent));
    }
    mArrows.arrows.transform();
  }

  //----------------------------------------------------------------------
  /**
  */
  double* SurfaceRegionRepresentation::GetBounds()
  {
    BuildRepresentation();
    mBoundingBox->SetBounds(mSphere.actor->GetBounds());
    mBoundingBox->AddBounds(mPoly.selectedPoints->GetBounds());
    return mBoundingBox->GetBounds();
  }

  /**
  */
  void SurfaceRegionRepresentation::setArrowColor(const Vec3d& color)
  {
    mArrows.arrows.setColor(color);
  }

  //----------------------------------------------------------------------
  /**
  */
  void SurfaceRegionRepresentation::CreateDefaultProperties()
  {
    mOutlineProperty->SetAmbient(0);
    mOutlineProperty->SetColor(Colors::Gray);

    mSphere.propActive->SetColor(Colors::Green);
    mSphere.propInactive->SetColor(Colors::Orange);

    mLine.propActive->SetColor(Colors::Green);
    mLine.propActive->SetLineWidth(2.0);
    mLine.propInactive->SetColor(Colors::Orange);
    mLine.propInactive->SetLineWidth(2.0);

    mSphere.actor->SetProperty(mSphere.propActive);
    mLine.actor->SetProperty(mLine.propActive);
    mArrows.arrows.getActor()->SetProperty(mArrows.propActive);

    mPoly.selectedPointsActor->GetProperty()->SetPointSize(4);
    mPoly.selectedPointsActor->GetProperty()->SetColor(0,0,1);
  }

  //----------------------------------------------------------------------
  /**
  */
  void SurfaceRegionRepresentation::setCache(vtkInteractorObserver* pObj)
  {
    mPoly.callback->mCache = pObj;
  }

  //----------------------------------------------------------------------
  /** \brief sets the interaction state and highlights appropriate parts of representation
  */
  void SurfaceRegionRepresentation::SetInteractionState(int state)
  {
    InteractionState = state;
    Highlight(state);
  }

  /**
  */
  void SurfaceRegionRepresentation::Highlight(int state)
  {
    switch (state)
    {
      case enMoveSphere:
        HighlightSphere(1);
        break;
      case enScrollSphere:
        HighlightSphere(1);
        break;
      default:
        HighlightSphere(0);
    }
  }

  /**
  */
  int  SurfaceRegionRepresentation::ComputeInteractionState(int X, int Y, int modify)
  {
    // Okay, we can process this. Try to pick handles first;
    if (!Renderer || !Renderer->IsInViewport(X, Y))
    {
      InteractionState = enOutside;
      return InteractionState;
    }
    // Try and pick a handle first
    mLastPicker    = nullptr;
    mCurrentHandle = nullptr;
    vtkAssemblyPath* path = GetAssemblyPath(X, Y, 0., mHandlePicker);
    if (path != nullptr)
    {
      ValidPick = 1;
      mLastPicker = mHandlePicker;
      mCurrentHandle = reinterpret_cast<vtkActor *>(path->GetFirstNode()->GetViewProp());
      if (mCurrentHandle == mSphere.actor)
      {
        InteractionState = enMoveSphere;
      }
    }
    return InteractionState;
  }

  /**
  */
  void SurfaceRegionRepresentation::WidgetInteraction(double e[2])
  {
    // Convert events to appropriate coordinate systems
    vtkCamera *camera = Renderer->GetActiveCamera();
    if (!camera)
      return;
    double focalPoint[4], pickPoint[4], prevPickPoint[4];
    double z, vpn[3];
    camera->GetViewPlaneNormal(vpn);
    // Compute the two points defining the motion vector
    double pos[3];
    mHandlePicker->GetPickPosition(pos);
    vtkInteractorObserver::ComputeWorldToDisplay(Renderer, pos[0], pos[1], pos[2],focalPoint);
    z = focalPoint[2];
    // Process the motion
    switch (InteractionState)
    {
      case enMoveSphere:
        vtkInteractorObserver::ComputeDisplayToWorld(Renderer, mLastEventPosition[0], mLastEventPosition[1], z, prevPickPoint);
        vtkInteractorObserver::ComputeDisplayToWorld(Renderer, e[0], e[1], z, pickPoint);
        moveSphere(prevPickPoint, pickPoint);
        break;
      case enScrollSphere:
      {
        const auto diff = mScaleRation*(e[1] - mLastEventPosition[1]);
        scrollSphere(diff);
        break;
      }
      case enPlaceSphere:
      { 
        placeSphere(e);
        break;
      }
    }
    // Store the start position
    mLastEventPosition[0] = e[0];
    mLastEventPosition[1] = e[1];
    mLastEventPosition[2] = 0.0;
  }

  /**
  */
  void SurfaceRegionRepresentation::placeSphere(double* e)
  {
    // Pick from this location.
    auto picker = make_vtk<vtkPropPicker>();
    picker->Pick(e[0], e[1], 0, Renderer);
    if (picker->GetActor() == nullptr)
      return;
    double* pos = picker->GetPickPosition();
    mLine.line->SetPoint1(pos);
    mSphere.sphere->SetCenter(pos);
    Modified();
    BuildRepresentation();
  }

  /**
  */
  void SurfaceRegionRepresentation::moveSphere(double* p1, double* p2)
  {
    double* center = mSphere.sphere->GetCenter();
    double v[3];
    for (int i(0); i<3; ++i)
    {
      v[i] = center[i] + (p2[i] - p1[i]);
    }
    mLine.line->SetPoint1(v);
    mSphere.sphere->SetCenter(v);
    Modified();
    BuildRepresentation();
  }

  /**
  */
  void SurfaceRegionRepresentation::scrollSphere(double dist)
  {
    const auto pSphere = Vec3d(mSphere.sphere->GetCenter());
    const auto pCam    = Vec3d(Renderer->GetActiveCamera()->GetPosition());
    const auto v       = pSphere-pCam;
    auto pNew = pCam + (dist + v.norm())*v.normalized();
    mSphere.sphere->SetCenter(pNew.data());
    mLine.line->SetPoint1(pNew.data());
    Modified();
    BuildRepresentation();
  }

  /**
  */
  void SurfaceRegionRepresentation::ReleaseGraphicsResources(vtkWindow* w)
  {
    mSphere.actor->ReleaseGraphicsResources(w);
    mLine.actor->ReleaseGraphicsResources(w);
    mPoly.selectedPointsActor->ReleaseGraphicsResources(w);
    mArrows.arrows.getActor()->ReleaseGraphicsResources(w);
  }

  /**
  */
  int  SurfaceRegionRepresentation::RenderOpaqueGeometry(vtkViewport* v)
  {
    int count(0);
    if (mSphere.actor->GetVisibility())
      count += mSphere.actor->RenderOpaqueGeometry(v);
    if (mLine.actor->GetVisibility())
      count += mLine.actor->RenderOpaqueGeometry(v);
    if (mPoly.selectedPointsActor->GetVisibility())
      count += mPoly.selectedPointsActor->RenderOpaqueGeometry(v);
    if (mArrows.arrows.getActor()->GetVisibility())
      count += mArrows.arrows.getActor()->RenderOpaqueGeometry(v);
    return count;
  }

  /**
  */
  int  SurfaceRegionRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport* v)
  {
    int count(0);
    if (mSphere.actor->GetVisibility())
      count += mSphere.actor->RenderTranslucentPolygonalGeometry(v);
    if (mLine.actor->GetVisibility())
      count += mLine.actor->RenderTranslucentPolygonalGeometry(v);
    if (mPoly.selectedPointsActor->GetVisibility())
      count += mPoly.selectedPointsActor->RenderTranslucentPolygonalGeometry(v);
    if (mArrows.arrows.getActor()->GetVisibility())
      count += mArrows.arrows.getActor()->RenderTranslucentPolygonalGeometry(v);
    return count;
  }

  /**
  */
  int  SurfaceRegionRepresentation::HasTranslucentPolygonalGeometry()
  {
    int count(0);
    if (mSphere.actor->GetVisibility())
      count |= mSphere.actor->HasTranslucentPolygonalGeometry();
    if (mLine.actor->GetVisibility())
      count |= mLine.actor->HasTranslucentPolygonalGeometry();
    if (mPoly.selectedPointsActor->GetVisibility())
      count |= mPoly.selectedPointsActor->HasTranslucentPolygonalGeometry();
    if (mArrows.arrows.getActor()->GetVisibility())
      count |= mArrows.arrows.getActor()->HasTranslucentPolygonalGeometry();
    return count;
  }

  /**
  */
  void SurfaceRegionRepresentation::HighlightSphere(int highlight)
  {
    if ( highlight )
    {
      mSphere.actor->SetProperty(mSphere.propActive);
      mLine.actor->SetProperty(mLine.propActive);
    }
    else
    {
      mSphere.actor->SetProperty(mSphere.propInactive);
      mLine.actor->SetProperty(mLine.propInactive);
    }
  }
   
  /** \brief satisfy the requirements of vtk's interace
  */
  void SurfaceRegionRepresentation::PlaceWidget(const Vec3d& newCenter)
  {
    auto* b = GetBounds();
    const auto oldCenter = 0.5*(Vec3d(b[0], b[2], b[4]) + Vec3d(b[1], b[3], b[5]));
    const auto v = newCenter-oldCenter;
    double newBounds[6];
    for (int i(0); i<3; ++i)
    {
      newBounds[2*i]   = b[2*i]   + v[i]; // min
      newBounds[2*i+1] = b[2*i+1] + v[i]; // max
    }
    PlaceWidget(newBounds);
  }

  /**
  */
  void SurfaceRegionRepresentation::PlaceWidget(double* b)
  {
    const auto p = 0.5*(Vec3d(b[0], b[2], b[4]) + Vec3d(b[1], b[3], b[5]));

    // whatever...
    double bounds[6], center[3];
    AdjustBounds(b, bounds, center);

    mSphere.sphere->SetCenter(p.x(), p.y(), p.z());

    for (int i(0); i<6; ++i)
    {
      InitialBounds[i] = bounds[i];
    }
    InitialLength = sqrt((bounds[1]-bounds[0])*(bounds[1]-bounds[0]) +
      (bounds[3]-bounds[2])*(bounds[3]-bounds[2]) +
      (bounds[5]-bounds[4])*(bounds[5]-bounds[4]));

    ValidPick = 1; //since we have set up widget
    SizeHandles();
  }

  /**
  */
  Vec3d SurfaceRegionRepresentation::getDirectionAncor() const
  {
    return Vec3d(mSphere.sphere->GetCenter());
  }

  /**
  */
  std::vector<Vec3d> SurfaceRegionRepresentation::getSurfacePoints() const
  {
    std::vector<Vec3d> res;
    const auto N = mPoly.selectedPoints->GetNumberOfPoints();
    for (vtkIdType i(0); i<N; ++i)
    {
      Vec3d next;
      mPoly.selectedPoints->GetPoint(i, next.data());
      res.push_back(next);
    }
    return res;
  }

  //----------------------------------------------------------------------------
  /** \brief Switches handles (the sphere and the button) off by manipulating the underlying
  * actor visibility.
  */
  void SurfaceRegionRepresentation::setHandles(bool on)
  {
    mLine.actor->SetVisibility(on && mPoly.selectedPoints->GetNumberOfPoints() > 0);
    mSphere.actor->SetVisibility(on);
    mPoly.selectedPointsActor->SetVisibility(on);
    mArrows.arrows.getActor()->SetVisibility( ! on);
    // the widget is either visible and evaluates events or is completely turned off
    mPoly.buttonWidget->SetEnabled(on);
    mPoly.buttonWidget->SetProcessEvents(on);
    // Force placement of buttonwidget
    placeButtonWidget();
    BuildRepresentation();
  }

  //----------------------------------------------------------------------------
  /**
  */
  void SurfaceRegionRepresentation::SizeHandles()
  {
    //double *center = static_cast<vtkDoubleArray *>(mPoints->GetData())->GetPointer(1);
    double radius = vtkWidgetRepresentation::SizeHandlesInPixels(1.5,mSphere.sphere->GetCenter());
    mSphere.sphere->SetRadius(radius);
  }

}
}

namespace
{
  void CreateImage(vtkSmartPointer<vtkImageData> image,
    unsigned char* color1,
    unsigned char* color2)
  {
    // Specify the size of the image data
    image->SetDimensions(10, 10, 1);
    image->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    int* dims = image->GetDimensions();

    // Fill the image with
    for (int y = 0; y < dims[1]; y++)
    {
      for (int x = 0; x < dims[0]; x++)
      {
        unsigned char* pixel =
          static_cast<unsigned char*>(image->GetScalarPointer(x, y, 0));
        if (x < 5)
        {
          pixel[0] = color1[0];
          pixel[1] = color1[1];
          pixel[2] = color1[2];
        }
        else
        {
          pixel[0] = color2[0];
          pixel[1] = color2[1];
          pixel[2] = color2[2];
        }
      }
    }
  }
}