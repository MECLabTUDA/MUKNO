#pragma once
#include "muk_common_api.h"

#include "MukVector.h"

#include <vector>
#include <memory>

namespace gris
{
  namespace muk
  { 
    /** \brief A polygon on a plane in 3D

      creates a polygon in 2D by computing a mapping to the 2D x-y-plane
      A query for a 3D point projects the point on the x-y-plane and test there, if the point is inside.
    */
    class MUK_COMMON_API Polygon3D
    {
      public:
        Polygon3D();
        ~Polygon3D();
        
      public:
        void create(const std::vector<Vec3d>& v);
        bool isInside(const Vec3d& p);

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
