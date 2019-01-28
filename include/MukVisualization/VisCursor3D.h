#pragma once

#include "VisAbstractObject.h"

#include "MukCommon/MukState.h"

class vtkArrowSource;
class vtkTransform;
class vtkTransformPolyDataFilter;

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API VisCursor3D : public VisAbstractObject
    {
      public:
        explicit VisCursor3D(const std::string& name);
        virtual ~VisCursor3D();

      public:
        void setPosition(const Vec3d& p);
        void setDirection(const Vec3d& p);
        void setState(const MukState& p);

        Vec3d getPosition() const;
        Vec3d getDirection() const;

        bool arrowIsVisible() const;
        void setArrowVisible(bool b);
        
      private:
        void computeArrow();

      private:
        bool mShowArrow;
        MukState mState;
        vtkSmartPointer<vtkArrowSource> mpArrowSource;
        vtkSmartPointer<vtkTransform>   mpArrowTransform;
        vtkSmartPointer<vtkTransformPolyDataFilter> mpArrowFilter;
    };

  }
}
