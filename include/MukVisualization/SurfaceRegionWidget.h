#pragma once
#include "muk_visualization_api.h"

#include <vtkInteractionWidgetsModule.h> // For export macro
#include <vtkAbstractWidget.h>
#include <vtkSmartPointer.h>
class vtkHandleWidget;

namespace gris
{
  namespace muk
  {
    class SurfaceRegionRepresentation;

    /** \brief 

      See BoxWidget for extended documentary on the principal work flow
    */
    class MUK_VIS_API SurfaceRegionWidget : public vtkAbstractWidget
    {
      protected:
        SurfaceRegionWidget();
        SurfaceRegionWidget(const SurfaceRegionWidget&) = delete;
        SurfaceRegionWidget& operator=(const SurfaceRegionWidget&) = delete;
        ~SurfaceRegionWidget();

      public:
        static SurfaceRegionWidget *New();
        vtkTypeMacro(SurfaceRegionWidget, vtkAbstractWidget);
        void PrintSelf(ostream& os, vtkIndent indent);

      public:
        void SetRepresentation(SurfaceRegionRepresentation*r) {this->Superclass::SetWidgetRepresentation(reinterpret_cast<vtkWidgetRepresentation*>(r)); }
        void CreateDefaultRepresentation();
        
        void SetEnabled(int on);
        void SetProcessEvents(int on);

      protected:
        // These methods handle events
        static void SelectAction(vtkAbstractWidget*);
        static void EndSelectAction(vtkAbstractWidget*);
        static void ScrollAction(vtkAbstractWidget*);
        static void MoveAction(vtkAbstractWidget*);
        // represent internal state
        enum EnWidgetState {Start=0,Active};

      protected:
        int mWidgetState; /// Manage the state of the widget
    };
  }
}