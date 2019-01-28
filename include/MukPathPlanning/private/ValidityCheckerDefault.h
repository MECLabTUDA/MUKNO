#pragma once

#include "private/IStateValidityChecker.h"

#include <memory>

namespace gris
{
  namespace muk
  {

    class ValidityCheckerDefault : public IStateValidityChecker
    {
      public:
      ValidityCheckerDefault(ompl::base::SpaceInformation* si) : IStateValidityChecker(si) {}
      ValidityCheckerDefault(const ompl::base::SpaceInformationPtr& si) : IStateValidityChecker(si) {}

      virtual ~ValidityCheckerDefault() {}

      virtual void setCollisionDetector(std::shared_ptr<ICollisionDetector> pObj) {}
      virtual bool isValid(const ompl::base::State* state) const { return true; }
    };

  }
}
