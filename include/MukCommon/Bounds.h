#pragma once

#include "muk_common_api.h"
#include "IBounds.h"

#include "MukVector.h"

#include <algorithm>

namespace gris
{
namespace muk
{

  /**
  */
  class MUK_COMMON_API Bounds : public IBounds
  {
    public:
      Bounds();

    public:
      virtual bool isInside(const MukState& state) const;
      virtual bool isInside(const Vec3d& position) const;

    public:
      void         setMax(const Vec3d& p)                { mMax = p; }
      void         setMax(double x, double y, double z)  { mMax.x() = x; mMax.y()=y; mMax.z() = z; }
      void         setMin( const Vec3d& p)               { mMin = p; }
      void         setMin(double x, double y, double z)  { mMin.x() = x; mMin.y()=y; mMin.z() = z; }
      Vec3d        getMin() const { return mMin; }
      Vec3d        getMax() const { return mMax; }

    public:
      void update(double* d);

    private:
      Vec3d mMin;
      Vec3d mMax;
  };

}
}
