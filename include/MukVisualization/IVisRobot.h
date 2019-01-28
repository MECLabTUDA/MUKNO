#pragma once

#include "VisAbstractObject.h"

#include "MukCommon/MukTransform.h"

namespace gris
{
  namespace muk
  {
    class MukState;

    /**
    */
    class MUK_VIS_API IVisRobot : public VisAbstractObject
    {
      public:
        IVisRobot();
        virtual ~IVisRobot();

        virtual const char* name() const = 0;
        
        virtual void loadModel() = 0;

      public:
        virtual void setState(const MukState& state) = 0;
        virtual void setTransform(vtkTransform* pTransform) = 0;
    };

}
}
