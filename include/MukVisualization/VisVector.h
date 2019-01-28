#pragma once

#include "VisScaleableObject.h"

#include <vtkArrowSource.h>

namespace gris
{
  namespace muk
  {
    class MUK_VIS_API VisVector : public VisScaleableObject
    {
      public:
        explicit VisVector(const std::string& name);
        virtual ~VisVector();

      public:
        virtual void update();
        void setVector(const Vec3d& from, const Vec3d& normal);

      public:
        vtkSmartPointer<vtkArrowSource> getArrowSource();

      private:
        vtkSmartPointer<vtkArrowSource> mpArrow;

    };
  }
}
