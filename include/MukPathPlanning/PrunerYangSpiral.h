#pragma once

#include "muk_pathplanning_api.h"

#include "MukCommon/IPathOptimizer.h"

namespace gris
{
  namespace muk
  {
    /** \brief A path optimizer that optimizes only length and curvature by throwing away unnecessary waypoints
    */
    class MUK_PP_API PrunerYangSpiral : public IPathOptimizer
    {
      public:
        PrunerYangSpiral();
        ~PrunerYangSpiral();

      public:
        static const char* s_name()       { return "Yang-Spiral-Pruner"; }
        const char* name()          const { return s_name(); }

      public:
        virtual MukPath calculate(const MukPath& input) const;

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
