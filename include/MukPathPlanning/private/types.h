#pragma once

#pragma warning (push)
#pragma warning( disable: 4267 ) // forcing value to bool
#pragma warning( disable: 4800 ) // forcing value to bool
#pragma warning( disable: 4996 ) // function (localtime) may be unsafe
#include <ompl/base/spaces/SE3StateSpace.h>
#pragma warning (pop)

namespace ompl
{
  namespace base
  {
  }
  namespace geometric
  {
    using MukStateSpace = ompl::base::SE3StateSpace;
    using MukStateType  = MukStateSpace::StateType;
  }
}

namespace gris
{
namespace muk
{
  namespace ob = ompl::base;
  namespace og = ompl::geometric;
}
}
