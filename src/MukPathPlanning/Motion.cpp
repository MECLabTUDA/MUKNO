#include "private/muk.pch"
#include "private/Motion.h"


namespace ompl
{
namespace geometric
{

  /** \brief creates a state and lets the user deallocate the state's memory
  */
  Motion* Motion::create(const base::SpaceInformationPtr& si)
  {       
    auto m = std::make_unique<Motion>();
    m->pState = si->allocState();
    return m.release();
  }

  /** \brief creates a state with automated memory allocation
  */
  std::unique_ptr<Motion, Motion::deleter_type> Motion::make_unique(const base::SpaceInformationPtr& si)
  { 
    std::unique_ptr<Motion, deleter_type> result(new Motion(), [&] (Motion* p) { si->freeState(p->pState); delete p; });
    result->pState = si->allocState();
    return result;
  }
}
}