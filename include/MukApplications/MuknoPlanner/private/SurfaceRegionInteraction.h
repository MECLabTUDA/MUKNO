#pragma once
#include "private/IMukInteraction.h"

#include "MukCommon/MukVector.h"

namespace gris
{
  namespace muk
  {
    class VisSurfaceRegion;

    /**
    */
    class SurfaceRegionInteraction : public IMukInteraction
    {
      public:
        static SurfaceRegionInteraction* New();
        vtkTypeMacro(SurfaceRegionInteraction, IMukInteraction);

      public:
        SurfaceRegionInteraction();
        
      public:
        // VTK API
        virtual void OnRightButtonUp();

        // Mukno
        virtual void initialize(VisSurfaceRegion* pObj = nullptr);
        virtual void finalize();

      private:
        enum EnType
        {
          enAsStart,
          enAsGoal,
        };

      private:
        void finishInteraction();
        void accept(EnType type);
        void discard();

      private:
        VisSurfaceRegion* mpRegion = nullptr;
    };
  }
}
