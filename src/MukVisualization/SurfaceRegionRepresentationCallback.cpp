#include "private/muk.pch"
#include "private/SurfaceRegionRepresentationCallback.h"

#include "MukCommon/MukVector.h"
#include "MukCommon/muk_common.h"

#include "MukVisualization/PolyDataHandler.h"

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCoordinate.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkImplicitFunction.h>
#include <vtkInteractorStyleDrawPolygon.h>
#include <vtkLine.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPropPicker.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>


namespace
{
  const double MAGIC_VTK_ENGINE_SCALAR = 0.999999;
  using namespace gris;
  void minimize(const int* newPoint, Vec2i& p)
  {
    for (int i(0); i<2; ++i)
      if (newPoint[i] < p[i])
        p[i] = newPoint[i];
  }

  void maximize(const int* newPoint, Vec2i& p)
  {
    for (int i(0); i<2; ++i)
      if (newPoint[i] > p[i])
        p[i] = newPoint[i];
  }

  /**
  */
  struct Polygon
  {
    using K =  CGAL::Exact_predicates_inexact_constructions_kernel;
    using Point = K::Point_2;

    void create(const std::vector<Vec2d>& v)
    {
      mPoints.clear();
      std::transform(v.begin(), v.end(), std::back_inserter(mPoints), [&] (const auto& p) { return Point(p.x(), p.y()); });
    }

    bool isInside(const Vec2d& p)
    {
      const auto query = Point(p.x(), p.y());
      switch(CGAL::bounded_side_2(mPoints.begin(), mPoints.end(), query, K()))
      {
        case CGAL::ON_BOUNDED_SIDE :
        case CGAL::ON_BOUNDARY:
          return true;
          break;
        case CGAL::ON_UNBOUNDED_SIDE:
          return false;
          break;
      }
      return false;
    }

    bool isInside(const Vec2i& p)
    {
      return isInside(Vec2d(p.x(), p.y()));
    }

    std::vector<Point> mPoints;
  };

  /**
  */
  class ImplicitPolygon : public vtkImplicitFunction
  {
    public:
      static ImplicitPolygon* New();
      vtkTypeMacro(ImplicitPolygon, vtkImplicitFunction);

    public:
      void createPolygon(const std::vector<Vec2d>& polygon)
      {
        mPoly.create(polygon);
      }

      void setRenderer(vtkRenderer* ren) { mRenderer = ren; }

    public:
      virtual double EvaluateFunction(double x[3])
      {
        using namespace gris::muk;
        auto coord = make_vtk<vtkCoordinate>();
        coord->SetCoordinateSystemToWorld();
        coord->SetValue(x);
        Vec2i tmp (coord->GetComputedViewportValue(mRenderer));
        return mPoly.isInside(tmp) ? -1 : +1;
      }

      virtual void EvaluateGradient(double x[3], double g[3])
      {
        for (int i(0); i<3; ++i)
          g[i] = 0;
      }

    private:
      Polygon mPoly;
      vtkRenderer* mRenderer;
  };

  vtkStandardNewMacro(ImplicitPolygon);
}

namespace gris
{
namespace muk
{
  /**
  */
  SurfaceRegionRepresentationCallback::SurfaceRegionRepresentationCallback()
  {
    drawer = make_vtk<vtkInteractorStyleDrawPolygon>();
    drawer->AddObserver(vtkCommand::SelectionChangedEvent, this);
  }

  /**
  */
  void SurfaceRegionRepresentationCallback::Execute(vtkObject* caller, unsigned long ev, void*)
  {
    switch (ev)
    {
      case vtkCommand::StateChangedEvent:
        executeStateChangedEvent(caller);
        break;
      case vtkCommand::SelectionChangedEvent:
        executeSelectionChangedEvent(caller);
        break;
    }
  }

  /**
  */
  void SurfaceRegionRepresentationCallback::executeStateChangedEvent(vtkObject* caller)
  {
    auto* buttonWidget = reinterpret_cast<vtkButtonWidget*>(caller);
    auto* rep = reinterpret_cast<vtkTexturedButtonRepresentation2D*>(buttonWidget->GetRepresentation());
    int state = rep->GetState();
    if (state)
    {
      mInteractor->SetInteractorStyle(drawer);
    }
    else
    {
      int i = mCache == nullptr;
      mInteractor->SetInteractorStyle(mCache);
    }
  }

  /**
  */
  void SurfaceRegionRepresentationCallback::executeSelectionChangedEvent(vtkObject* caller)
  {
    auto polygon2d = drawer->GetPolygonPoints();
    // retrive actor
    if (!mInteractor)
    {
      return;
    }
    auto pPicker = make_vtk<vtkPropPicker>();
    if (polygon2d.empty())
    {
      return;
    }
    pPicker->Pick(polygon2d.front().GetX(), polygon2d.front().GetY(), 0, mRenderer);
    mPickedActor = pPicker->GetActor();
    if (nullptr == mPickedActor)
    {
      LOG_LINE << "no surface picked";
      return;
    }
    // convert painted polygon to 3D
    std::vector<Vec2d> viewPortPolygon;
    for (const auto& p : polygon2d)
    {
      viewPortPolygon.push_back(Vec2d(p.GetX(), p.GetY()));
    }
    // extract surfaces within polygon
    auto extractGeometry = make_vtk<vtkExtractPolyDataGeometry>();
    auto isInsidePolygon = make_vtk<ImplicitPolygon>();
    isInsidePolygon->createPolygon(viewPortPolygon);
    isInsidePolygon->setRenderer(mRenderer);
    extractGeometry->SetImplicitFunction(isInsidePolygon);
    extractGeometry->SetInputData(mPickedActor->GetMapper()->GetInput());
    extractGeometry->SetExtractInside(1);
    extractGeometry->Update();
    auto glyphFilter = make_vtk<vtkVertexGlyphFilter>();
    glyphFilter->SetInputConnection(extractGeometry->GetOutputPort());
    glyphFilter->Update();
    // compute if lines between points and camera intersect with the polydata
    auto* camera = mRenderer->GetActiveCamera();
    double pos[3];
    camera->GetPosition(pos);
    const auto N = glyphFilter->GetOutput()->GetNumberOfPoints();
    auto queryLine = make_vtk<vtkPolyData>();
    {
      auto points = make_vtk<vtkPoints>();
      points->InsertNextPoint(pos);
      points->InsertNextPoint(pos);
      auto line = make_vtk<vtkLine>();
      line->GetPointIds()->SetId(0, 0);
      line->GetPointIds()->SetId(1, 1);
      auto lines = make_vtk<vtkCellArray>();
      lines->InsertNextCell(line);
      queryLine->SetPoints(points);
      queryLine->SetLines(lines);
    }

    auto coord = make_vtk<vtkCoordinate>();
    coord->SetCoordinateSystemToWorld();
    std::vector<bool> isVisible(N,false);
    mRenderer->Render();
    for (vtkIdType i(0); i<N; ++i)
    {
      coord->SetValue(glyphFilter->GetOutput()->GetPoint(i));
      auto* displayCoord = coord->GetComputedViewportValue(mRenderer);
      auto z = mRenderer->GetZ(displayCoord[0], displayCoord[1]);

      double display[3];
      display[0] = displayCoord[0];
      display[1] = displayCoord[1];
      display[2] = z;

      mRenderer->SetDisplayPoint (display);
      mRenderer->DisplayToWorld ();
      auto* world = mRenderer->GetWorldPoint ();
      const auto d1 = (Vec3d(pos)-Vec3d(world)).squaredNorm();
      const auto d2 = (Vec3d(pos)-Vec3d(glyphFilter->GetOutput()->GetPoint(i))).squaredNorm();
      const double eps = 0.1;
      if ( d1+eps > d2 )//|| z>= MAGIC_VTK_ENGINE_SCALAR)
        isVisible[i] = true;
    }

    // add points of the active actor if they are visible
    auto newPoints = make_vtk<vtkPoints>();
    for (vtkIdType i(0); i<glyphFilter->GetOutput()->GetNumberOfPoints(); ++i)
    {
      if (isVisible[i])
        newPoints->InsertNextPoint(glyphFilter->GetOutput()->GetPoint(i));
    }
    mSetPointsCallback(newPoints);

    auto* rep = reinterpret_cast<vtkTexturedButtonRepresentation2D*>(mButton->GetRepresentation());
    rep->NextState();
    mButton->InvokeEvent(vtkCommand::StateChangedEvent, nullptr);
    mButton->Render();
  }
}
}