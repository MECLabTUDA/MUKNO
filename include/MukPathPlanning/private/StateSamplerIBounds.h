#pragma once
#include "private/types.h"

#include "MukCommon/IBounds.h"

#pragma warning (push)
#pragma warning( disable: 4267 ) // ompl -> conversion from size_t to unsigned int 
#include <ompl/base/StateSampler.h>
#include <ompl/util/RandomNumbers.h>
#pragma warning (pop)

#include <memory>

namespace gris
{
  namespace muk
  {
    /** \brief 
    */
    class StateSamplerIBounds : public ob::StateSampler
    {
      public:
        static ob::StateSamplerPtr create(const ob::StateSpace *si, const gris::muk::IBounds* pBounds);

      public:
        explicit StateSamplerIBounds(const ob::StateSpace *si, const gris::muk::IBounds* pBounds);
        virtual ~StateSamplerIBounds();

        void setBounds(const IBounds& bounds);

        /** \brief Sample a state */
        virtual void sampleUniform(ob::State *state);

        /** \brief Sample a state near another, within specified distance */
        virtual void sampleUniformNear(ob::State *state, const ob::State *near, const double distance);

        /** \brief Sample a state using a Gaussian distribution with given \e mean and standard deviation (\e stdDev) */
        virtual void sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev);

      private:
        const gris::muk::IBounds* mpBounds;
        Vec3d mMax;
        Vec3d mMin;
    };
  }
}
