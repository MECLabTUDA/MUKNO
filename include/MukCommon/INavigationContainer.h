#pragma once

#include "muk_common_api.h"

#include "INavigator.h"
#include "MukPath.h"
#include "IInterpolator.h"
#include "gstd/dynamicProperty.h"

namespace gris
{
  namespace muk {
    class MUK_COMMON_API INavigationContainer
    {
    public:
      virtual std::shared_ptr<ProtectedNavigator> getNavigator()         = 0;
      virtual NavigatorProperty*                  getProperties()        = 0;
      std::string                                 getNavigatorName()  const { return const_cast<INavigationContainer*>(this)->getNavigator()->name(); }

	    static  const char* s_name()     { return "Navigation";  }
	    virtual const char* name() const { return s_name(); }

    public:
      virtual enNavigatorState  navigatorState() = 0;
      virtual enConsumerState   consumerState()  = 0;
      // the Thread has left its run-Function()
      virtual bool              isTerminated()   = 0;
    };
  }
}
