#include "private/muk.pch"
#include "PlaneStateRegion.h"

#include "muk_common.h"
#include "MukException.h"
#include "polygon_3d_tools.h"
#include "StateRegionFactory.h"

#include "gris_math.h"

#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>

BOOST_CLASS_EXPORT(gris::muk::PlaneStateRegion);

namespace gris
{
  namespace muk
  {
    REGISTER_REGION(PlaneStateRegion);

    /**
    */
    PlaneStateRegion::PlaneStateRegion()
      : mResolution(1.0)
    {
    }

    /** \brief Computes states on the plane within the bounding polygon
    */
    std::vector<MukState> PlaneStateRegion::getStates() const
    {
      std::vector<MukState> result;
      // compute maximum distance to a point on the polygon
      double dist = 0;
      for (const auto& p : mBoundingPolygon)
      {
        const auto d = (mOrigin-p).norm();
        if (d>dist)
          dist = d;
      }
      // create the polygon
      Polygon3D poly;
      poly.create(mBoundingPolygon);
      // traverse a rectangular grid on the plane with the given resolution and the width 'dist'
      // compute for each grid point, if it is inside the polygon and if so, create a new state from it
      const auto rand1 = Vec3d(1,0,0);
      const auto rand2 = Vec3d(0,1,0);
      auto v1 = rand1.cross(mNormal).normalized();
      if (v1.hasNaN())
        v1 = rand2.cross(mNormal).normalized();
      const auto v2 = v1.cross(mNormal).normalized();
      const auto N = std::max(1, static_cast<int>( dist / mResolution + 0.5));
      const auto dirFlag = mPointsTowardsAncor ? 1 : -1;
      for (int i(-N); i<=N; ++i)
        for (int j(-N); j<=N; ++j)
      {
        MukState next;
        next.coords  = mOrigin + i*mResolution*v1 + j*mResolution*v2;
        if (poly.isInside(next.coords))
        {
          next.tangent = dirFlag * (mDirectionAncor - next.coords).normalized();
          result.push_back(next);
        }
      }
      return result;
    }


    /**
    */
    template<>
    void PlaneStateRegion::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive& ar, const unsigned version)
    {
      // http://stackoverflow.com/questions/14146393/boosts-load-construct-data-and-serializing-derived-classes-through-pointer-to-b
      boost::serialization::void_cast_register<PlaneStateRegion, IStateRegion>();
      //ar & boost::serialization::base_object<IStateRegion>(*this); // instead of this
      ar & mOrigin & mNormal & mDirectionAncor & mBoundingPolygon & mResolution;
    }

    /**
    */
    template<>
    void PlaneStateRegion::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive& ar, const unsigned version)
    {
      boost::serialization::void_cast_register<PlaneStateRegion, IStateRegion>();
      ar & mOrigin & mNormal & mDirectionAncor & mBoundingPolygon & mResolution;
    }
  }
}
