#include "private/muk.pch"
#include "Bounds.h"
#include "MukVector.h"
#include "overload_deduction.h"

#include <functional>

namespace gris
{
  namespace muk
  { 
    using namespace gris::gstd;
    /**
    */
    Bounds::Bounds()
    {
      mMax.fill( 0 );
      mMin.fill( 0 );
      
      declareProperty<Vec3d>("Max",
          std::bind(SELECT<const Vec3d&>::OVERLOAD_OF(&Bounds::setMax), this, std::placeholders::_1), 
          std::bind(&Bounds::getMax, this));
      declareProperty<Vec3d>("Min",
        std::bind(SELECT<const Vec3d&>::OVERLOAD_OF(&Bounds::setMin), this, std::placeholders::_1), 
        std::bind(&Bounds::getMin, this));
    }
 
    /**
    */
    void Bounds::update(double* d)
    {
      if (nullptr!=d)
      {
        mMin.x() = std::min(mMin.x(), d[0]);
        mMin.y() = std::min(mMin.y(), d[2]);
        mMin.z() = std::min(mMin.z(), d[4]);
        mMax.x() = std::max(mMax.x(), d[1]);
        mMax.y() = std::max(mMax.y(), d[3]);
        mMax.z() = std::max(mMax.z(), d[5]);
      }
    }

    bool Bounds::isInside(const MukState& state) const
    {
      return isInside(state.coords);
    }

    bool Bounds::isInside(const Vec3d& position) const
    {
      return mMin.x() < position.x() && mMax.x() > position.x()
        &&   mMin.y() < position.y() && mMax.y() > position.y()
        &&   mMin.z() < position.z() && mMax.z() > position.z();
    }

  }
}