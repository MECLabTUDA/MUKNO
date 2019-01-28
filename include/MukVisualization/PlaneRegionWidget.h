#pragma once
#include "muk_visualization_api.h"

#include <vtkInteractionWidgetsModule.h> // For export macro
#include <vtkImplicitPlaneWidget2.h>
#include <vtkSmartPointer.h>

namespace gris
{
  namespace muk
  {
    class PlaneRegionRepresentation;

    /** \brief 

    See BoxWidget for extended documentary on the principal work flow
    */
    class MUK_VIS_API PlaneRegionWidget : public vtkImplicitPlaneWidget2
    {
      protected:
        PlaneRegionWidget();
        PlaneRegionWidget(const PlaneRegionWidget&) = delete;
        PlaneRegionWidget& operator=(const PlaneRegionWidget&) = delete;
        ~PlaneRegionWidget();

      public:
        static PlaneRegionWidget *New();
        vtkTypeMacro(PlaneRegionWidget, vtkImplicitPlaneWidget2);

      public:
        void SetRepresentation(PlaneRegionRepresentation *r) { Superclass::SetWidgetRepresentation(reinterpret_cast<vtkWidgetRepresentation*>(r)); }
        void CreateDefaultRepresentation();
        void SetProcessEvents(int on);

      public:
        static void SelectAction(vtkAbstractWidget*);
        static void ScaleAction(vtkAbstractWidget*);
    };
  }
}