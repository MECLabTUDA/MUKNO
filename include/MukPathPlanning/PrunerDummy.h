#pragma once

#include "muk_pathplanning_api.h"

#include "MukCommon/IPathPruner.h"

namespace gris
{
  namespace muk
  {
    class MUK_PP_API PrunerDummy : public IPathPruner
    {
      public:
        PrunerDummy();
        ~PrunerDummy();

      public:
        static const char* s_name()         { return "PrunerDummy"; }
               const char* name()     const { return s_name(); }

      public:
        virtual MukPath calculate(const MukPath& input);
        void    clone(const IPathPruner* pOther);
    };
  }
}
