#pragma once

#include "muk_visualization_api.h"

#include "MukCommon/MukVector.h"

#include "gstd/DynamicProperty.h"

#include <iostream>

namespace gris
{
  namespace muk
  {
    /**
    */
    struct MUK_VIS_API CameraConfiguration : public gstd::DynamicProperty
    {
      public:
        CameraConfiguration();

        std::ostream& operator<<(std::ostream& os);
        std::istream& operator>>(std::istream& os);

      public:
        void         setPosition(const Vec3d& p)          { position = p; }
        const Vec3d& getPosition()                  const { return position; }
        void         setFocalPoint(const Vec3d& p)        { focalPoint = p; }
        const Vec3d& getFocalPoint()                const { return focalPoint; }
        void         setViewUp(const Vec3d& p)            { viewUp = p; }
        const Vec3d& getViewUp()                    const { return viewUp; }


      private:
        Vec3d position;
        Vec3d focalPoint;
        Vec3d viewUp;
    };
  }
}

namespace std
{
  MUK_VIS_API std::ostream& operator<<(std::ostream& os, const gris::muk::CameraConfiguration& config);
  MUK_VIS_API std::istream& operator>>(std::istream& is, gris::muk::CameraConfiguration& config);
}