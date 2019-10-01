#pragma once

// declare template spezialization of type text-archive
// this allows definition in cpp
#define Forward_Declare_Muk_Boost_Serialization \
namespace boost { \
namespace serialization { \
  class access; \
} \
namespace archive{ \
  class text_iarchive; \
  class text_oarchive; \
} \
}

#define Use_Muk_Boost_Serialization \
friend class boost::serialization::access; \
template<typename Archive> \
void serialize(Archive& ar, const unsigned version); \
template<> \
void serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive& ar, const unsigned version); \
template<typename Archive> \
void serialize(Archive& ar, const unsigned version) const; \
template<> \
void serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive& ar, const unsigned version);\

/* macros for easy use of gstd::DynamicProperty
// 
// use like this:
// e.g. declareProperty<double>("SurfaceValue", MUK_SET(double, setSurfaceValue), MUK_GET(getSurfaceValue));
// e.g. declareProperty<Vec3d>("Sigma", MUK_C_SET(Vec3d, setSigma), MUK_GET(getSigma));
*/
#define MUK_SET(type, set) [&] (type d) { set(d); }
#define MUK_GET(get) [&] () { return get(); }
#define MUK_C_SET(type, set) [&] (const type& d) { set(d); }
#define MUK_C_GET(type, get) [&] () -> const type& { return get(); }
