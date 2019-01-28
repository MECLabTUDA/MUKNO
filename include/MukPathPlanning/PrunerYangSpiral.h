#pragma once

#include "muk_pathplanning_api.h"

#include "MukCommon/IPathPruner.h"

namespace gris
{
  namespace muk
  {
    class MUK_PP_API PrunerYangSpiral : public IPathPruner
    {
      public:
        PrunerYangSpiral();
        ~PrunerYangSpiral();

      public:
        static const char* s_name()         { return "Yang-Spiral-Pruner"; }
        const char* name()     const { return s_name(); }

      public:
        virtual MukPath calculate(const MukPath& input);
        virtual void clone(const IPathPruner* pOther);

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
