#include "private/muk.pch"
#include "IStateRegion.h"

#include <boost/serialization/export.hpp>

BOOST_SERIALIZATION_ASSUME_ABSTRACT(gris::muk::IStateRegion)
//BOOST_CLASS_EXPORT(gris::muk::IStateRegion);

namespace gris
{
namespace muk
{
  /**
  */
  IStateRegion::~IStateRegion()
  {
  }

  /**
  */
  template<>
  void IStateRegion::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive& ar, const unsigned version)
  {
  }

  /**
  */
  template<>
  void IStateRegion::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive& ar, const unsigned version)
  {
  }

}
}