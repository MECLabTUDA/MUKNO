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
 
    /** \brief increases the bounding box so the passed boundingbox lies within.

      \param[in] d pointer to six doubles (xmin, xmax, ymin, ymax, zmin, zmax)
    */
    void Bounds::update(double d[6])
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

    /** \brief increases the bounding box so the passed point lies within.

      \param[in] p a 3D point
    */
    void Bounds::update(const Vec3d& p)
    {
      mMin.x() = std::min(mMin.x(), p.x());
      mMin.y() = std::min(mMin.y(), p.y());
      mMin.z() = std::min(mMin.z(), p.z());
      mMax.x() = std::max(mMax.x(), p.x());
      mMax.y() = std::max(mMax.y(), p.y());
      mMax.z() = std::max(mMax.z(), p.z());
    }

    /** \brief returns if the position of the passed state lies within the boundingbox.
    */
    bool Bounds::isInside(const MukState& state) const
    {
      return isInside(state.coords);
    }

    /** \brief returns if the position of the passed point lies within the boundingbox.
    */
    bool Bounds::isInside(const Vec3d& position) const
    {
      return mMin.x() < position.x() && mMax.x() > position.x()
        &&   mMin.y() < position.y() && mMax.y() > position.y()
        &&   mMin.z() < position.z() && mMax.z() > position.z();
    }

  }
}