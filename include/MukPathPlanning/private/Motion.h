#pragma once

#include <ompl/base/SpaceInformation.h>

#include <memory>

namespace ompl
{
  namespace base
  {
    class State;
  }

  namespace geometric
  {
    /**
    */
    struct Motion
    {
      using deleter_type = std::function<void(Motion*)>;
      
      Motion()
        : pState(nullptr) 
        , pParent(nullptr) 
      {
      }
      
      static Motion* create(const base::SpaceInformationPtr& si);
      static std::unique_ptr<Motion, deleter_type> make_unique(const base::SpaceInformationPtr& si);

      base::State* pState;
      Motion*      pParent;
    };
  }
}