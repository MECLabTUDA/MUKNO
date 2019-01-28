#pragma once

#include "private/IStateValidityChecker.h"
#include "private/KdTreeConfig.h"

#include "MukCommon/IBounds.h"

#include <memory>

namespace gris
{
  namespace muk
  {

    class ValidityCheckerKdTreeNearestNeighbor : public IStateValidityChecker
    {
      public:
        ValidityCheckerKdTreeNearestNeighbor(ompl::base::SpaceInformation* si);
        ValidityCheckerKdTreeNearestNeighbor(const ompl::base::SpaceInformationPtr& si);
        virtual ~ValidityCheckerKdTreeNearestNeighbor();
        
      public:
        void    setMaxDistance(double);
        double  getMaxDistance()        const;

      public:
        virtual bool   isValid(const ompl::base::State* state) const;
        virtual double clearance(const ompl::base::State*) const;

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };

  }
}
