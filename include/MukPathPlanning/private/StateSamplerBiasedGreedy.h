#pragma once

#include <ompl/base/StateSampler.h>
#include <ompl/util/RandomNumbers.h>


namespace ob = ompl::base;

namespace gris
{
  namespace muk
  {
    class BiasedGreedyStateSampler : public ob::StateSampler
    {
      public:
        static ob::StateSamplerPtr create(const ob::StateSpace *si);

      public:
        explicit BiasedGreedyStateSampler(const ob::StateSpace *si);
        virtual ~BiasedGreedyStateSampler() {}
        
        /** \brief Sample a state */
        virtual void sampleUniform(ob::State *state);

        /** \brief Sample a state near another, within specified distance */
        virtual void sampleUniformNear(ob::State *state, const ob::State *near, const double distance);

        /** \brief Sample a state using a Gaussian distribution with given \e mean and standard deviation (\e stdDev) */
        virtual void sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev);                
    };
  }
}
