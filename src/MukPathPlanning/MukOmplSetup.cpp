#include "private/muk.pch"
#include "private/MukOmplSetup.h"
#include "private/StateSamplerIBounds.h"
#include "private/types.h"
#include "private/ValidityCheckerKdTreeNearestNeighbor.h"

#include "MukCommon/IBounds.h"
#include "MukCommon/ICollisionDetector.h"

#pragma warning (push)
#pragma warning( disable: 4267 ) // ompl -> conversion from size_t to unsigned int 
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#pragma warning (pop)

namespace gris
{
namespace muk
{
  MukOmplSetup::MukOmplSetup()
    : mpValidityChecker(nullptr)
    , mpGoal(nullptr)
  {
    mpProbDef.reset();
    mpStateSampler.reset();
    mpValidityChecker = nullptr;
    mpSpaceInfo.reset();
    mpSpace.reset();

    // setup ompl::StateSpace / StateInformation
    mpSpace = std::make_shared<og::MukStateSpace>();
    {
      // bounds checking will be done by the custom IStateValidityChecker
      // so set this here to the (useless) maximum, as valid ompl bounds are required by the library
      ob::RealVectorBounds bounds(3);        
      bounds.setLow(-1000000); // numeric_limits doesn't work... great...
      bounds.setHigh(1000000);
      mpSpace->as<og::MukStateSpace>()->setBounds(bounds);
    }
    mpSpaceInfo = std::make_shared<ob::SpaceInformation>(mpSpace);
    auto pObj = std::make_shared<ValidityCheckerKdTreeNearestNeighbor>(mpSpaceInfo);
    mpValidityChecker = pObj.get();
    mpSpaceInfo->setStateValidityChecker(std::static_pointer_cast<ompl::base::StateValidityChecker>(pObj));
    mpSpaceInfo->setup();
    mpProbDef = std::make_shared<ob::ProblemDefinition>(mpSpaceInfo);
    mpGoal = std::make_shared<ob::MukGoalStates>(mpSpaceInfo);
    mpProbDef->setGoal(mpGoal);
  }

  /**
  */
  void MukOmplSetup::initialize(std::shared_ptr<ICollisionDetector> pDetector)
  {
    auto& obj = dynamic_cast<ValidityCheckerKdTreeNearestNeighbor&>(*mpValidityChecker);
    obj.setCollisionDetector(pDetector);
  }

  /**
  */
  void MukOmplSetup::initialize(const IBounds& pBounds)
  {
    mpSpace->setStateSamplerAllocator( std::bind(&StateSamplerIBounds::create, std::placeholders::_1, &pBounds) );
  }
}
}