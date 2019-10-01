#pragma once

#include "muk_pathplanning_api.h"

#include "MukCommon/IPathOptimizer.h"

namespace gris
{
  namespace muk
  {
    /** \brief Dummy interface that just returns the input path.
    */
    class MUK_PP_API PrunerDummy : public IPathOptimizer
    {
      public:
        PrunerDummy();
        ~PrunerDummy();

      public:
        static const char* s_name()         { return "PrunerDummy"; }
               const char* name()     const { return s_name(); }

      public:
        virtual MukPath calculate(const MukPath& input) const;
    };
  }
}
