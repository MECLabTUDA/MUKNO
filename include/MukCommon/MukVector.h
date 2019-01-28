#pragma once
#include "gstd/Vector.h"

#include <boost/test/auto_unit_test.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/array.hpp>

#include <vector>

namespace boost {
  namespace serialization {

    template<class Archive, class T, class S, size_t N>
    void serialize(Archive & ar, gris::Vector<T, S, N> & a, const unsigned int version)
    {
      ar & boost::serialization::make_array(a.data(), N);
    }

  } // namespace serialization
} // namespace boost

namespace gris
{
  namespace muk
  {
    typedef std::vector<Vec3d> VecN3d;

    using Vec4d = gris::Vector<double, double, 4>;
  }
}