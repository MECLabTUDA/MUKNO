#include "private/muk.pch"
#include "MukState.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

namespace gris
{
namespace muk
{ 
  /**
  */
  MukState::MukState()
    : coords(0,0,0)
    , tangent(1,0,0)
  {
  }

  /**
  */
  MukState::MukState(const Vec3d& c, const Vec3d& t) 
    : coords(c)
    , tangent(t)
  {
  }

  /**
  */
  MukState::MukState(const MukState& o)
    : coords(o.coords)
    , tangent(o.tangent)
  {
  }

  /**
  */
  MukState& MukState::operator=(const MukState& o)
  {
    if (this != &o)
    {
      coords  = o.coords;
      tangent = o.tangent;
    }
    return *this;
  }

  /**
  */
  MukState::MukState(MukState&& o)
    : coords(std::move(o.coords))
    , tangent(std::move(o.tangent))
  {
  }

  MukState::~MukState()
  {
  }

  /**
  */
  MukState& MukState::operator=(MukState&& o)
  {
    if (this != &o)
    {
      coords = std::move(o.coords);
      tangent = std::move(o.tangent);
    }
    return *this;
  }


  void MukState::swap(MukState& o)
  {
    using std::swap;
    swap(coords, o.coords);
    swap(tangent, o.tangent);
  }

  void swap(MukState& a, MukState& b)
  {
    a.swap(b);
  }

  std::ostream& operator<<(std::ostream& os, const MukState& s)
  {
    return os << s.coords << " " << s.tangent;
  }

  /** \brief compares if position and direction are equal
  */
  bool MukState::operator== (const MukState& rhs) const
  {
    return coords == rhs.coords && tangent==rhs.tangent;
  }

  /** \brief compares if position and direction are unequal
  */
  bool MukState::operator!= (const MukState& rhs) const
  {
    return ! (rhs==*this);
  }
  
  /**
  */
  template<>
  void MukState::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive& ar, const unsigned version)
  {
    ar & coords & tangent;
  }

  /**
  */
  template<>
  void MukState::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive& ar, const unsigned version)
  {
    ar & coords & tangent;
  }
}
}


namespace std
{
  template<>
  void swap <gris::muk::MukState> (gris::muk::MukState& a, gris::muk::MukState& b)
  {
    a.swap(b);
  }
}