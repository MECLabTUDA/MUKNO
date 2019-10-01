#pragma once

#include "muk_common.h"
#include "muk_common_api.h"
#include "MukState.h"

#include "gstd/DynamicProperty.h"

Forward_Declare_Muk_Boost_Serialization

namespace gris
{
  namespace muk
  {
    /** \brief Abstract representation for an initial or goal region of a motion planning problem definition.

      Major interface for derived classes is the #getStates method, which is used by motion planner to extract the states.
    */
    class MUK_COMMON_API IStateRegion : public gstd::DynamicProperty
    {
      public:
        IStateRegion() = default;
        IStateRegion(const IStateRegion&) = default;
        IStateRegion& operator=(const IStateRegion&) = default;
        IStateRegion(IStateRegion&&) = default;
        IStateRegion& operator=(IStateRegion&&) = default;
        virtual ~IStateRegion();
        
        virtual const char* name() const = 0;

      public:
        virtual std::vector<MukState>         getStates() const = 0;
        virtual std::unique_ptr<IStateRegion> clone()     const = 0;

      private:
        Use_Muk_Boost_Serialization
    };

  }
}
