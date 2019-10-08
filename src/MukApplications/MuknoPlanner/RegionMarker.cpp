#include "private/muk.pch"
#include "private/RegionMarker.h"

#include "MukCommon/geometry.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/PolyDataHandler.h"

#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkAbstractPicker.h>

#include <itkPolygonSpatialObject.h>

#pragma warning (push)
#pragma warning (disable: 4996)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#pragma warning (pop)

#include <omp.h>

namespace {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2 Point;
  typedef CGAL::Polygon_2<K> Polygon_2;
}

namespace gris
{
  namespace muk
  {
    /**
    */
    RegionMarker::RegionMarker(vtkInteractorStyle* pStyle)
      : mpInteractorStyle(pStyle)
      , mpRenderer(nullptr)
      , mRegionModeFlag(inactive)
    {
      mpPolyData  = make_vtk<vtkPolyData>();
      mpActor     = make_vtk<vtkActor>();
      mpMapper    = make_vtk<vtkPolyDataMapper>();
      auto points = make_vtk<vtkPoints>();      
      {
        mpPolyData->SetPoints(points);
        mpMapper->SetInputData(mpPolyData);
        mpActor->SetMapper(mpMapper);
        mpActor->GetProperty()->SetColor(gris::muk::Colors::Red);
        mpActor->GetProperty()->SetPointSize(5.0);
      }

      mpPolyDataPoly  = make_vtk<vtkPolyData>();
      mpActorPoly     = make_vtk<vtkActor>();
      mpMapperPoly    = make_vtk<vtkPolyDataMapper>();
      points = make_vtk<vtkPoints>();      
      {
        mpPolyDataPoly->SetPoints(points);
        mpMapperPoly->SetInputData(mpPolyDataPoly);
        mpActorPoly->SetMapper(mpMapperPoly);
        mpActorPoly->GetProperty()->SetColor(gris::muk::Colors::Red);
        mpActorPoly->GetProperty()->SetPointSize(5.0);
      }

      //mpActor->GetProperty()->SetPointSize(5.0);      
    }

    /**
    */
    RegionMarker::~RegionMarker()
    {
      if (mpRenderer != nullptr)
      {
        //mpRenderer->RemoveActor(mpActor);
        //mpRenderer->RemoveActor(mpActorPoly);
      }
    }

    /**
    */
    void RegionMarker::setRenderer(vtkRenderer* pRenderer)
    {
      if (mpRenderer != nullptr)
      {
        mpRenderer->RemoveActor(mpActor);
        mpRenderer->RemoveActor(mpActorPoly);
      }
      mpRenderer = pRenderer;
      mpRenderer->AddActor(mpActor);
      mpRenderer->AddActor(mpActorPoly);
    }

    /**
    */
    void RegionMarker::setType(const char c)
    {
      if (c == 'm')
      {
        if (mRegionModeFlag != mark)
        {
          mRegionModeFlag = mark;
          LOG_LINE << "mark region activated";
        }
        else
        {
          mRegionModeFlag = inactive;
          LOG_LINE << "mark region deactivated";
        }
      }
      if (c == 'u')
      {
        if (mRegionModeFlag != unmark)
        {
          mRegionModeFlag = unmark;
          LOG_LINE << "unmark region activated";
        }
        else
        {
          mRegionModeFlag = inactive;
          LOG_LINE << "unmark region deactivated";
        }
      }
      if (c == 'c')
      {
        mRegionModeFlag = inactive;
        mTmpPoints.clear();
        mPoints.clear();
        setPoints();
        LOG_LINE << "cleared marked region";
      }
    }

    /**
    */
    void RegionMarker::processPolygon()
    {
      std::vector<Point> points;      
      std::transform(mPolygon.begin(), mPolygon.end(), std::back_inserter(points), [&] (const Vec2d& p) { return Point(p.x(), p.y()); });
      Polygon_2 poly(points.begin(), points.end());
      auto bbox = poly.bbox();
      
      std::vector<Vec3d> points3D;
      auto* pInter = mpInteractorStyle->GetInteractor();
      // stupid linker shit, some missing library or whatever. anyway, use itk instead of poly::bounded_side below....
      const unsigned int Dimension = 2;
      typedef itk::PolygonSpatialObject< Dimension > PolygonType;
      typedef double PixelType;
      PolygonType::Pointer polygon = PolygonType::New();
      std::for_each(mPolygon.begin(), mPolygon.end(), [&] (const Vec2d& p) 
        {
          PolygonType::PointType point;
          point[0] = p.x();
          point[0] = p.y();
          polygon->AddPoint( point );
        });
      
      const int N = static_cast<int>( bbox.xmax()-bbox.xmin() + 0.5 );
      const double rest = bbox.xmax() - (int)bbox.xmax();
      //#pragma omp parallel for
      for (int i=0; i<N; ++i)
      {
        for (double j(-bbox.ymin()); j<=bbox.ymax(); ++j)
        {           
          //if (CGAL::ON_BOUNDED_SIDE == poly.bounded_side(Point(i, j)))
          PolygonType::PointType point;
          point[0] = i+rest;
          point[0] = j;
          if (polygon->IsInside(point))
          {
            pInter->GetPicker()->Pick(pInter->GetEventPosition()[0]+i+rest, pInter->GetEventPosition()[1]+j, 0, mpRenderer);
            double picked[3];
            pInter->GetPicker()->GetPickPosition(picked);
            points3D.push_back( Vec3d(picked) );
          }
        }
      }

      switch (mRegionModeFlag)
      {
        case mark:
          markPoints(points3D);
          break;
        case unmark:
          unmarkPoints(points3D);
          break;
      }
      setPoints();
    }

    /**
    */
    void RegionMarker::processPoints()
    {
      Circle2D circle;
      fitCircle(mCirclePoints, circle);
      const Vec2d llc = circle.getCenter() - Vec2d(circle.getRadius());
      const Vec2d urc = circle.getCenter() + Vec2d(circle.getRadius());
      auto* pInter = mpInteractorStyle->GetInteractor();      
      const int    MaxCols = 10;
      const double stepX =  (urc.x() - llc.x()) / MaxCols;
      const int    MaxRows = 10;
      const double stepY =  (urc.y() - llc.y()) / MaxRows;

      //const int N = static_cast<int>( MaxCols );
      //#pragma omp parallel for
      for (int i=0; i<MaxCols; ++i)
      {
        for (int j=0; j<MaxRows; ++j)
        {
          const double x = llc.x() + i*stepX;
          const double y = llc.y() + j*stepY;
          if (circle.isInside( Vec2d(x,y) ))
          {
            pInter->GetPicker()->Pick(x, y, 0, mpRenderer);
            double picked[3];
            pInter->GetPicker()->GetPickPosition(picked);
            mPoints.push_back( Vec3d(picked) );
          }
        }
      }
      mTmpPoints.clear();
      setPoints();
      mRegionModeFlag = inactive;
    }

    /**
    */
    void RegionMarker::addPoint(const Vec3d& p)
    {
      mTmpPoints.push_back(p);
    }

    /**
    */
    void RegionMarker::addPoint(const Vec2d& p)
    {
      mCirclePoints.push_back(p);
    }


    /**
    */
    void RegionMarker::markPoints(const std::vector<Vec3d>& v)
    {
      std::copy(v.begin(), v.end(), std::back_inserter(mPoints));
      std::unique(mPoints.begin(), mPoints.end());
    }

    /**
    */
    void RegionMarker::unmarkPoints(const std::vector<Vec3d>& v)
    {
      std::vector<Vec3d>::iterator newEnd;
      for_each(v.begin(), v.end(), [&] (const Vec3d& p)
      {
        newEnd = std::remove_if(mPoints.begin(), mPoints.end(), [&] (const Vec3d& q) { return p==q; });
      });
      mPoints.erase(newEnd, mPoints.end());
    }

    /**
    */
    void RegionMarker::addPolygonPoint(const Vec2d& p)
    {
      if (mPolygon.size() < 2)
      {
        mPolygon.push_back(p);
        setPolygon();
        return;
      }
      const double Diff = 3.0;
      auto iter = std::find_if(mPolygon.begin(), mPolygon.end(), [&] (const Vec2d& q) { return abs(q.x()-p.x()) < Diff && abs(q.y()-p.y()) < Diff; });
      if (iter!=mPolygon.end())
      {
        setPoints();
        mRegionModeFlag = inactive;
      }
      else
      {
        mPolygon.push_back(p);
        setPolygon();
      }
    }

    /**
    */
    void RegionMarker::clear()
    {
      mPoints.clear();
    }

    /**
    */
    std::vector<MukState> RegionMarker::getRegion()
    {
      std::vector<MukState> result(mPoints.size());
      std::transform(mPoints.begin(), mPoints.end(), result.begin(), [&] (const Vec3d& p) { return MukState(p, Vec3d(1,0,0)); });
      LOG_LINE << "size region " << result.size();
      return result;
    }

    /**
    */    
    void RegionMarker::setPoints()
    {
      auto points = make_vtk<vtkPoints>();
      std::for_each(mPoints.begin(), mPoints.end(), [&] (const Vec3d& p) { points->InsertNextPoint(p.data()); });
      mpPolyData->SetPoints(points);
      PolyDataHandler::addVertices(mpPolyData.Get());
      mpMapper->SetInputData(mpPolyData);
      mpMapper->Update();
    }

    /**
    */
    void RegionMarker::setPolygon()
    {
      /*auto points = make_vtk<vtkPoints>();
      std::for_each(mPolygon.begin(), mPolygon.end(), [&] (const Vec3d& p)
      {
        points->InsertNextPoint(p.x, p.y, p.z); 
      });
      mpPolyData->SetPoints(points);
      PolyDataHandler::addVertices(mpPolyData.Get());
      mpMapper->SetInputData(mpPolyData);
      mpMapper->Update();*/
    }
  }
}