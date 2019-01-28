#pragma once

#include "VisAbstractObject.h"
#include "MukCommon/MukPath.h"

namespace gris
{
  namespace muk
  {
    class MUK_VIS_API VisTrajectory : public VisAbstractObject
    {
      public:
        explicit VisTrajectory(const std::string& name);
        ~VisTrajectory();

      public:
        void setData(const MukPath& path);
        virtual void update();

      private:
        MukPath mTrajectory;
    };
  }
}
