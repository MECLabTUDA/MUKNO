#pragma once
#include "BaseController.h"

#include "gstd/DynamicProperty.h"

#include <vtkSmartPointer.h>

namespace gris
{
  namespace muk
  {
    class IMukInteraction;
    class MultiPortSphereRegionInteraction;
    class PlaneRegionInteraction;
    class SphereRegionInteraction;
    class SurfaceRegionInteraction;
    class VisStateRegion;
    class VtkWindow;

    /**
    */
    class InteractionController : public BaseController
    {
      public:
        InteractionController();
        virtual ~InteractionController() = default;

      public:
        virtual void initialize();
        virtual void setupConnections();

        void setDefaultInteraction();
        void setInteraction(const std::string& key);
        void setInteraction(VisStateRegion& region);
        void showClosestPoint();

      private:
        void setup(SphereRegionInteraction& vis);
        void setup(SurfaceRegionInteraction& vis);
        void setup(MultiPortSphereRegionInteraction& vis);
        void setup(PlaneRegionInteraction& vis);

      private:
        VtkWindow* mpWindow;
        vtkSmartPointer<IMukInteraction> mpInteractorStyle;
    };
  }
}
