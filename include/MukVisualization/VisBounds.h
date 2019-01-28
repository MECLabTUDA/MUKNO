#pragma once

#include "VisAbstractObject.h"

#include "MukCommon/MukVector.h"

namespace gris
{
  namespace muk
  {
    class IBounds;
    /** \brief Visualization of class "Bounds"
    */
    class MUK_VIS_API VisBounds : public VisAbstractObject
    {
      public:
        VisBounds();
        ~VisBounds();

      public:
        virtual void update();

      public:
        void setBounds(const IBounds& bounds);

      private:
        Vec3d mMin;
        Vec3d mMax;
    };

  }
}
