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
    /**
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
        virtual std::vector<MukState> getStates() const = 0;

      private:
        Use_Muk_Boost_Serialization
    };

  }
}
