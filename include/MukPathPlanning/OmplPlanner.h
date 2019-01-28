#pragma once

#include "muk_pathplanning_api.h"

#include "MukCommon/IPathPlanner.h"

#include <memory>

namespace gris
{
  namespace muk
  {
    struct MukOmplSetup;

    /** \brief wrapper class that handles the initialization/updating of ompl-objects that do now depend on the planner
    */
    class MUK_PP_API OmplPlanner : public IPathPlanner
    {
      public:
        OmplPlanner();
        virtual ~OmplPlanner();

      public:
        virtual void  initialize();
        virtual void  update(bool switched = false);
        virtual void  clearSearch();
        
      protected:
        std::unique_ptr<MukOmplSetup> mpSetup;
    };
  }
}