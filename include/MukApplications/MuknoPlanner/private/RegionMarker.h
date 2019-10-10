#pragma once

#include "MukCommon/MukState.h"
#include "MukCommon/MukVector.h"
#include "MukCommon/vtk_tools.h"

#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkInteractorStyle.h>

namespace gris
{
  namespace muk
  {

    class RegionMarker
    {
      public:
        enum EnMarkRegionType
        {
          inactive,
          mark,
          unmark
        };

      public:
        RegionMarker(vtkInteractorStyle* pStyle);
        ~RegionMarker();
        
      public:
        void setType(const char c);
        EnMarkRegionType getType() const { return mRegionModeFlag; }

      public:
        bool isActive() const { return mRegionModeFlag != inactive; }        
        void setRenderer(vtkRenderer* pRenderer);

        void addPolygonPoint(const Vec2d& p);
        void addPoint(const Vec3d& p);
        void addPoint(const Vec2d& p);

        void processPoints();
        void markPoints(const std::vector<Vec3d>& mPoints);
        void unmarkPoints(const std::vector<Vec3d>& mPoints);
        void clear();

        std::vector<MukState> getRegion();

      private:
        void setPoints();
        void setPolygon();
        void processPolygon();

      private:
        std::vector<Vec3d> mPoints;
        std::vector<Vec3d> mTmpPoints;
        std::vector<Vec2d> mCirclePoints;
        vtkInteractorStyle* mpInteractorStyle;
        EnMarkRegionType    mRegionModeFlag;

        std::vector<Vec2d> mPolygon;

        vtkRenderer*                mpRenderer;
        DeclVtk(vtkActor)           mpActor;
        DeclVtk(vtkPolyDataMapper)  mpMapper;
        DeclVtk(vtkPolyData)        mpPolyData;

        DeclVtk(vtkActor)           mpActorPoly;
        DeclVtk(vtkPolyDataMapper)  mpMapperPoly;
        DeclVtk(vtkPolyData)        mpPolyDataPoly;
    };

  }
}
