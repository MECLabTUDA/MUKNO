#include "private/muk.pch"
#include "SurfaceStateRegion.h"

#include "StateRegionFactory.h"

#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>

BOOST_CLASS_EXPORT(gris::muk::SurfaceStateRegion);

namespace gris
{
namespace muk
{
  REGISTER_REGION(SurfaceStateRegion);
  
  /**
  */
  SurfaceStateRegion::SurfaceStateRegion()
    : mPointsTowardsAncor(false)
  {
  }

  /** \brief Computes states on the plane within the bounding polygon
  */
  std::vector<MukState> SurfaceStateRegion::getStates() const
  {
    std::vector<MukState> result;
    for (const auto& p : mSurfacePoints)
    {
      MukState next;
      next.coords = p;
      next.tangent = (mDirectionAncor-p).normalized();
      if ( ! mPointsTowardsAncor)
        next.tangent *= -1;
      result.push_back(next);
    }
    return result;
  }


  /**
  */
  template<>
  void SurfaceStateRegion::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive& ar, const unsigned version)
  {
    // http://stackoverflow.com/questions/14146393/boosts-load-construct-data-and-serializing-derived-classes-through-pointer-to-b
    boost::serialization::void_cast_register<SurfaceStateRegion, IStateRegion>();
    ar & mSurfacePoints & mDirectionAncor & mPointsTowardsAncor;
  }

  /**
  */
  template<>
  void SurfaceStateRegion::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive& ar, const unsigned version)
  {
    boost::serialization::void_cast_register<SurfaceStateRegion, IStateRegion>();
    ar & mSurfacePoints & mDirectionAncor & mPointsTowardsAncor;
  }
}
}