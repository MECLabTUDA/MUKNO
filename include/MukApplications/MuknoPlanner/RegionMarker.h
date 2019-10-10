#pragma once

#include <vtkInteractorStyle.h>

namespace gris
{
  namespace muk
  {

    class RegionMarker
    {
      public:
        enum EnMarkRegionType
        {
          inactive,
          mark,
          unmark
        };

      public:
        RegionMarker(vtkInteractorStyle* pStyle);
        ~RegionMarker() {}

      public:
        void setType(const char c);
        EnMarkRegionType getType() const { return mRegionModeFlag; }
        
      private:
        vtkInteractorStyle* mpInteractorStyle;
        EnMarkRegionType    mRegionModeFlag;
        std::vector<Vec3d> mpPickedRegion;
    };

  }
}
