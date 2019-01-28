#include "private/muk.pch"
#include "muk_common.h"
#include "MukStateRegion.h"
#include "MukException.h"
#include "StateRegionFactory.h"

#include "Eigen/Dense"

#include "gris_math.h"

#include <boost/serialization/export.hpp>

namespace
{
  Eigen::Vector3d cast(const gris::Vec3d& v)
  {
    return Eigen::Vector3d(v.data());
  }

  gris::Vec3d cast(const Eigen::Vector3d& v)
  {
    return gris::Vec3d(v.data());
  }
}

BOOST_CLASS_EXPORT(gris::muk::MukStateRegion);

namespace gris
{
namespace muk
{
  REGISTER_REGION(MukStateRegion);

  /**
  */
  MukStateRegion::MukStateRegion()
    : mRadius(1)
    , mPhi(M_PI_2) // initial values correspond to half sphere
    , mResolution(20)
  {
    declareProperty<double>("Phi", MUK_SET(double, setPhi), MUK_GET(getPhi));
    declareProperty<double>("Radius", MUK_SET(double, setRadius), MUK_GET(getRadius));
    declareProperty<unsigned int>("Resolution", MUK_SET(unsigned int, setResolution), MUK_GET(getResolution));
  }
    
  /**
  */
  void MukStateRegion::setPhi(double val)
  {
    if (val < 0 || val > gris::M_PI)
    {
      std::string info = "Passed Value: " + std::to_string(val);
      throw MUK_EXCEPTION("Value of parameter phi should has to be in [0, pi]", info.c_str());
    }
    mPhi = val;
  }

  /** \brief computes the corresponding states on the sphere 

    create sphere, transform to center

    note the conflicting standards in geography, engineering, physics, mathematics, etc.
    (for convenience use the way of VTK)

    latitude:  phi   in (0, pi)   in radians (from north pole = (0,0,1) to south pole)
    longitude: theta in (0, 2*pi) in radians (from 'greenwich' = (1,0,0) around the sphere)
    use spherical coordinates 
      x = r cos(phi) cos(theta)
      y = r cos(phi) sin(theta)
      z = r sin(phi)

    Uniform distribution:
      I haven't found a solution to a uniform distribution of arbitrary presicion on parts of a sphere.
      This class/method makes the following approach: 
        - set the precision in direction of latitude.
        - compute the number of points on each "slice" of the sphere on a certain latitude such that the same precistion is realized
  */
  std::vector<MukState> MukStateRegion::getStates() const
  {
    using namespace Eigen;
    Transform<double, 3, 1> R;
    {
      auto zAxis = Vec3d(0, 0, 1);
      auto crossproduct = zAxis.cross(mCenter.tangent);
      double angle = std::acos(std::abs(mCenter.tangent.z()));
      if (mCenter.tangent.z() < 0)
      {
        angle = M_PI - angle;
      }
      auto axis  = cast(crossproduct);
      axis.normalize();
      R = AngleAxisd(angle, axis);
      auto tr  = Translation3d(Vector3d(cast(mCenter.coords)));
    }
    const double stepPhi    = mResolution == 0 ? 0 : mPhi / mResolution;
    std::vector<MukState> result;
    double phi(0);
    for (size_t i(0); i<=mResolution; ++i, phi+=stepPhi) // for each latitude ...
    {
      const double cphi = cos(phi);
      const double sphi = sin(phi);
      const size_t N    = mResolution == 0 || stepPhi < 10e-5 ? 0 : static_cast<size_t>( floor(cos(phi) / stepPhi) );
      const double stepTheta = N == 0 ? 0 : 2*gris::M_PI / N;
      for (size_t j(0); j<=N; ++j) // ... compute states on the circle (longitude)
      {
        MukState nextState;
        const double ctheta = cos(j * stepTheta);
        const double stheta = sin(j * stepTheta);
        auto point = Vector3d(sphi*ctheta, sphi*stheta, cphi);
        nextState.tangent = cast(R*point).normalized();
        nextState.coords = mCenter.coords;
        result.push_back(nextState);
        if (i==0)  // i==0 is equal to state mCenter
          break;
      }
    }
    return result;
  }

  /**
  */
  template<>
  void MukStateRegion::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive& ar, const unsigned version)
  {
    // http://stackoverflow.com/questions/14146393/boosts-load-construct-data-and-serializing-derived-classes-through-pointer-to-b
    boost::serialization::void_cast_register<MukStateRegion, IStateRegion>();
    //ar & boost::serialization::base_object<IStateRegion>(*this); // instead of this
    ar & mCenter & mRadius & mPhi & mResolution;
  }

  /**
  */
  template<>
  void MukStateRegion::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive& ar, const unsigned version)
  {
    boost::serialization::void_cast_register<MukStateRegion, IStateRegion>();
    ar & mCenter & mRadius & mPhi & mResolution;
  }
}
}
