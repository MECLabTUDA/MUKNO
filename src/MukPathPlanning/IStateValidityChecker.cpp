#include "private/muk.pch"
#include "private/IStateValidityChecker.h"

namespace gris
{
  namespace muk
  {
    IStateValidityChecker::IStateValidityChecker(ompl::base::SpaceInformation* si) 
      : StateValidityChecker(si)
      , mpBounds(nullptr) 
    {}

    IStateValidityChecker::IStateValidityChecker(const ompl::base::SpaceInformationPtr& si) 
      : StateValidityChecker(si)
    {}
  }
}