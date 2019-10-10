#pragma once
#include "private/IMukInteraction.h"

namespace gris
{
  namespace muk
  {
    class VisSphereRegion;

    /** \brief Generates the interaction to define a Initial or Goal Region consisting of a partial sphere and and arrow
    */
    class SphereRegionInteraction : public IMukInteraction
    {
      public:
        static SphereRegionInteraction* New();
        vtkTypeMacro(SphereRegionInteraction, IMukInteraction);

      public:
        SphereRegionInteraction();

      public:
        virtual void OnRightButtonUp();
        
        virtual void initialize(VisSphereRegion* pObj = nullptr);
        virtual void finalize();

      public:
        void setSphereRadius(double r);

      private:
        enum EnType
        {
          enAsStart,
          enAsGoal
        };

      private:
        void finishInteraction(); // return to DefaultInteraction3D
        void discard(); // return to DefaultInteraction3D
        void accept(EnType type);

      private:
        double mSphereRadius = 1.0;
        VisSphereRegion* mpRegion = nullptr;
    };
  }
}
