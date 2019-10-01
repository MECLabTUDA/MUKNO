#include "private/muk.pch"
#include "StateRegionSingleDirection.h"
#include "StateRegionFactory.h"

#include "MukState.h"

#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>

BOOST_CLASS_EXPORT(gris::muk::StateRegionSingleDirection);

namespace gris
{
namespace muk
{
  REGISTER_REGION(StateRegionSingleDirection);

  /**
  */
  StateRegionSingleDirection::~StateRegionSingleDirection()
  {
  }

  /**
  */
  std::vector<MukState> StateRegionSingleDirection::getStates() const
  {
    std::vector<MukState> result;
    std::transform(mPoints.begin(), mPoints.end(), std::back_inserter(result), [&] (const auto& p) { return MukState(p, mDirection); });
    return result;
  }

  /**
  */
  std::unique_ptr<IStateRegion> StateRegionSingleDirection::clone() const
  {
    auto pObj = std::make_unique<StateRegionSingleDirection>();
    pObj->mPoints     = mPoints;
    pObj->mDirection  = mDirection;
    return std::move(pObj);
  }

  /**
  */
  template<>
  void StateRegionSingleDirection::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive& ar, const unsigned version)
  {
    boost::serialization::void_cast_register<StateRegionSingleDirection, IStateRegion>();
    //ar & boost::serialization::base_object<IStateRegion>(*this);
    ar & mDirection & mPoints;
  }

  /**
  */
  template<>
  void StateRegionSingleDirection::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive& ar, const unsigned version)
  {
    boost::serialization::void_cast_register<StateRegionSingleDirection, IStateRegion>();
    //ar & boost::serialization::base_object<IStateRegion>(*this);
    ar & mDirection & mPoints;
  }
}
}