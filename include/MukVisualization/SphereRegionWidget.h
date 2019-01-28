#pragma once
#include "muk_visualization_api.h"

#include <vtkInteractionWidgetsModule.h> // For export macro
#include <vtkAbstractWidget.h>
#include <vtkSmartPointer.h>

namespace gris
{
  namespace muk
  {
    class SphereRegionRepresentation;

    /** \brief 

      See BoxWidget for extended documentary on the principal work flow
    */
    class MUK_VIS_API SphereRegionWidget : public vtkAbstractWidget
    {
      protected:
        SphereRegionWidget();
        SphereRegionWidget(const SphereRegionWidget&) = delete;
        SphereRegionWidget& operator=(const SphereRegionWidget&) = delete;
        ~SphereRegionWidget();

      public:
        static SphereRegionWidget *New();
        vtkTypeMacro(SphereRegionWidget, vtkAbstractWidget);

      public:
        void SetRepresentation(SphereRegionRepresentation *r) { Superclass::SetWidgetRepresentation(reinterpret_cast<vtkWidgetRepresentation*>(r)); }
        //SphereRegionRepresentation* representation() { return reinterpret_cast<SphereRegionRepresentation*>(this->WidgetRep); }
        void CreateDefaultRepresentation();

        void SetProcessEvents(int on);
        
        // These methods handle events
        static void SelectAction(vtkAbstractWidget*);
        static void EndSelectAction(vtkAbstractWidget*);
        static void MoveAction(vtkAbstractWidget*);
        //static void TranslateAction(vtkAbstractWidget*);

        // represent internal state
        enum EnWidgetState 
        {
          enStart = 0, 
          enActive 
        };
        /**
        * Update the cursor shape based on the interaction state. Returns 1
        * if the cursor shape requested is different from the existing one.
        */
        int UpdateCursorShape(int interactionState);

      protected:
        int mWidgetState;
    };
  }
}