#include "private/muk.pch"
#include "private/types.h"
#include "private/StateSamplerIBounds.h"

#include <Eigen/Dense>

#include <boost/make_shared.hpp>

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

namespace
{
  inline double sqr(double x)
  {
    return x*x;
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }   

  double angle(const Eigen::Vector3d& l, const Eigen::Vector3d& r)
  {
    using namespace Eigen;
    using namespace std;
    double nom = l.dot(r);
    double denom = l.norm() * r.norm();
    return denom == 0 ? 0 : acos(nom/denom);
  }
}

namespace gris
{
namespace muk
{
  /**
  */
  ob::StateSamplerPtr StateSamplerIBounds::create(const ob::StateSpace *si, const gris::muk::IBounds* pBounds)
  {
    ob::StateSamplerPtr output (new StateSamplerIBounds(si, pBounds));
    return output;
  }

  /**
  */
  StateSamplerIBounds::StateSamplerIBounds(const ob::StateSpace *si, const gris::muk::IBounds* pBounds)
    : StateSampler(si)
  {
    setBounds(*pBounds);
  }

  /**
  */
  StateSamplerIBounds::~StateSamplerIBounds()
  {
  }

  /**
  */
  void StateSamplerIBounds::setBounds(const IBounds& bounds)
  {
    mpBounds = &bounds;
    mMin = mpBounds->getMin();
    mMax = mpBounds->getMax();
  }

  /** \brief creates a sample within a rectangular box

     restrictions:
      - (x,y,z) in [bounds->min, bounds->max]
      - random unit quaternion
  */
  void StateSamplerIBounds::sampleUniform(ob::State *state)
  {
    using gris::Vec3d;
    typedef og::MukStateType Type;
    // sample location
    Vec3d position;
    do
    {
      position.x() = rng_.uniformReal(mMin.x(), mMax.x());
      position.y() = rng_.uniformReal(mMin.y(), mMax.y());
      position.z() = rng_.uniformReal(mMin.z(), mMax.z());
    }
    while( ! mpBounds->isInside(position));
    state->as<Type>()->setXYZ(position.x(), position.y(), position.z());
    // sample orientation
    double values[4];
    rng_.quaternion(values);
    state->as<Type>()->rotation().x = values[0];
    state->as<Type>()->rotation().y = values[1];
    state->as<Type>()->rotation().z = values[2];
    state->as<Type>()->rotation().w = values[3];
  }

  /**
  */
  void StateSamplerIBounds::sampleUniformNear(ob::State *state, const ob::State *near, const double distance)
  {
    typedef og::MukStateType Type;
    // get old values
    const double x = near->as<Type>()->getX();
    const double y = near->as<Type>()->getY();
    const double z = near->as<Type>()->getZ();

    const double rx = near->as<Type>()->rotation().x;
    const double ry = near->as<Type>()->rotation().y;
    const double rz = near->as<Type>()->rotation().z;
    const double rw = near->as<Type>()->rotation().w;

    // sample within a ball and same orientation
    double xyz[3];
    rng_.uniformInBall(distance, 3, xyz);
    state->as<Type>()->setXYZ(x+xyz[0], y+xyz[1], z+xyz[2]);
    state->as<Type>()->rotation().x = rx;
    state->as<Type>()->rotation().y = ry;
    state->as<Type>()->rotation().z = rz;
    state->as<Type>()->rotation().w = 0;
  }

  /**
  */
  void StateSamplerIBounds::sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev)
  {
  }

}
}