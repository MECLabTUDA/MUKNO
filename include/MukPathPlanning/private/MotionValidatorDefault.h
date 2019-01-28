#pragma once

#include "ompl/base/MotionValidator.h"

namespace ompl
{
  namespace base
  {
    /**
    */
    class MotionValidatorDefault : public ompl::base::MotionValidator
    {
      public:
        MotionValidatorDefault(SpaceInformation *si) : MotionValidator(si_) {}
        MotionValidatorDefault(const SpaceInformationPtr &si) : MotionValidator(si.get()) {}
        ~MotionValidatorDefault() {}

      public:
        bool checkMotion(const State *s1, const State *s2) const                                                { return true; }
        bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const          { return true; }
    };
  }
}