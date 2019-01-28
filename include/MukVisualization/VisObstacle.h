#pragma once

#include "VisAbstractObject.h"

#include "MukCommon/MukObstacle.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API VisObstacle : public VisAbstractObject
    {
      public:
        explicit VisObstacle(const MukObstacle& obs);
        ~VisObstacle();

      public:
        virtual void update() {}

      public:

      private:
    };
  }
}
