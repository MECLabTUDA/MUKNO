#pragma once

#include "muk_visualization_api.h"

#include "MukCommon/MukVector.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

#include <vector>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API Arrow
    {
      public:
        static Arrow create(const Vec3d& origin, const Vec3d& direction, double length = 1.0);

      public:
        Arrow(const Arrow& o) = default;
        Arrow& operator=(const Arrow& o) = default;
        ~Arrow() {}

      public:
        vtkSmartPointer<vtkPolyData> getData()       { return mpArrow; }
        vtkSmartPointer<vtkPolyData> getData() const { return mpArrow; }

      private:
        Arrow();
        
      private:
        vtkSmartPointer<vtkPolyData> mpArrow;
    };

    /**
    */
    class MUK_VIS_API ArrowTrajectory
    {
      public:
        ArrowTrajectory();

      public:
        void clear();
        void addArrow(Arrow a) { mArrows.emplace_back(a); }
        void transform();
        void setColor(const Vec3d& color);
        vtkSmartPointer<vtkActor> getActor() { return mpActor; }
        size_t size();

      private:
        std::vector<Arrow> mArrows;
        vtkSmartPointer<vtkActor> mpActor;
        vtkSmartPointer<vtkPolyDataMapper> mpMapper;
        vtkSmartPointer<vtkPolyData> mpData;
    };

    /**
    */
    class MUK_VIS_API VectorField
    {
      public:    
        using Trajectory = std::shared_ptr<ArrowTrajectory>;
        using Field      = std::vector<Trajectory>;

      public:
        void addTrajectory(Trajectory& pObj) { mTrajectories.push_back(pObj); pObj->transform(); }
        
        size_t            size()                  const { return mTrajectories.size(); }
        const Trajectory& getTrajectory(size_t i) const { return mTrajectories[i]; }
        Trajectory&       getTrajectory(size_t i)       { return mTrajectories[i]; }

      private:        
        Field mTrajectories;
    };
  }
}
