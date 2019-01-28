#pragma once

#include "muk_common_api.h"
#include "MukState.h"

#include "gstd/DynamicProperty.h"

namespace gris
{
  namespace muk
  {

    class MUK_COMMON_API IBounds : public gstd::DynamicProperty
    {
      public: 
        IBounds();
        virtual ~IBounds();
        
      public:
        virtual bool isInside(const MukState& state) const = 0;
        virtual bool isInside(const Vec3d& position) const = 0;

        virtual Vec3d    getMin() const = 0;
        virtual Vec3d    getMax() const = 0;
    };

  }
}
