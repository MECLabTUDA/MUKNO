#pragma once

#include "MukVector.h"

#include <Eigen/Dense>


namespace gris
{
  namespace muk
  {
    inline Eigen::Vector3d asEigen(const Vec3d& v)
    {
      return Eigen::Vector3d(v.data());
    }

    inline Vec3d asMuk(const Eigen::Vector3d& v)
    { 
      return Vec3d(v.data());
    }
  }
}
