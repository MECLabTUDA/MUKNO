#pragma once
#include "private/types.h"
#include "private/MukGoalStates.h"

#include "MukCommon/MukVector.h"

#pragma warning (push)
#pragma warning( disable: 4267 ) // forcing value to bool
#pragma warning( disable: 4800 ) // forcing value to bool
#pragma warning( disable: 4996 ) // function (localtime) may be unsafe
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#pragma warning (pop)

#include <memory>

namespace gris
{
  namespace muk
  {
    class IBounds;
    class ICollisionDetector;
    class IStateValidityChecker;
    class StateSamplerIBounds;

    /** \brief Helper class to setup the necessary ompl configurations
    */
    struct MukOmplSetup
    {
      MukOmplSetup();

      void initialize(std::shared_ptr<ICollisionDetector> pDetector);
      void initialize(const IBounds& pBounds);

      ob::StateSpacePtr         mpSpace;
      ob::SpaceInformationPtr   mpSpaceInfo;
      ob::ProblemDefinitionPtr  mpProbDef;
      std::shared_ptr<ob::MukGoalStates> mpGoal;
      IStateValidityChecker*    mpValidityChecker;
      std::shared_ptr<ob::StateSampler>   mpStateSampler;
    };
  }
}
