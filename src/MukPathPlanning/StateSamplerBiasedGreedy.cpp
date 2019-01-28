#include "private/muk.pch"
#include "private/types.h"
#include "private/StateSamplerBiasedGreedy.h"

#pragma warning (push)
#pragma warning( disable: 4800 ) // forcing value to bool
#pragma warning( disable: 4996 ) // function (localtime) may be unsafe
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#pragma warning (pop)

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
    BiasedGreedyStateSampler::BiasedGreedyStateSampler(const ob::StateSpace *si) 
      : StateSampler(si)
    {
      LOG_LINE << "BiasedGreedyStateSampler allocated";
    }

    void BiasedGreedyStateSampler::sampleUniform(ob::State *state)
    { 
      typedef og::MukStateType Type;
      // sample location
      const ob::RealVectorBounds& bounds = static_cast<const og::MukStateSpace*>(space_)->getBounds();
      const double x = rng_.uniformReal(bounds.low[0], bounds.high[0]);
      const double y = rng_.uniformReal(bounds.low[1], bounds.high[1]);
      const double z = rng_.uniformReal(bounds.low[2], bounds.high[2]);
      state->as<Type>()->setXYZ(x,y,z);
      // sample orientation
      double values[4];
      rng_.quaternion(values);
      state->as<Type>()->rotation().x = values[0];
      state->as<Type>()->rotation().y = values[1];
      state->as<Type>()->rotation().z = values[2];
      state->as<Type>()->rotation().w = values[3];            
    }

    /** \brief Sample a state near another, within specified distance */
    void BiasedGreedyStateSampler::sampleUniformNear(ob::State *state, const ob::State *near, const double distance)
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

      // sample within a ball and same orientation (2months later: omfg, why?)
      double xyz[3];
      rng_.uniformInBall(distance, 3, xyz);
      state->as<Type>()->setXYZ(x+xyz[0], y+xyz[1], z+xyz[2]);
      state->as<Type>()->rotation().x = rx;
      state->as<Type>()->rotation().y = ry;
      state->as<Type>()->rotation().z = rz;
      state->as<Type>()->rotation().w = 0;
    }

    /** \brief Sample a state using a Gaussian distribution with given \e mean and standard deviation (\e stdDev) */
    void BiasedGreedyStateSampler::sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev)
    {
      // never used by SplineBasedRRT
      sampleUniform(state);
    }
    
    ob::StateSamplerPtr BiasedGreedyStateSampler::create(const ob::StateSpace *si)
    {
      return std::make_shared<BiasedGreedyStateSampler>(si);
    }

    ob::StateSamplerPtr allocBiasedGreedyStateSampler(const ob::StateSpace *si)
    {
      return std::make_shared<BiasedGreedyStateSampler>(si);
    }

  }
}