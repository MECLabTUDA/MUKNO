#pragma once

#include "muk_visualization_api.h"
#include "VisStateRegion.h"

#include <memory>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API VisStateRegionFactory
    {
      public:
        static std::shared_ptr<VisStateRegion> create(const IStateRegion& region);
    };
  }
}
