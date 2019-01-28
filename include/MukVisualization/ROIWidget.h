#pragma once
#include "muk_visualization_api.h"

#include <vtkAbstractWidget.h>

namespace gris
{
  namespace muk
  {
    class ROIRepresentation;

    /**
    */
    class MUK_VIS_API ROIWidget : public vtkAbstractWidget
    {
      public:
        static ROIWidget *New();
        vtkTypeMacro(ROIWidget,vtkAbstractWidget);

        void PrintSelf(ostream& os, vtkIndent indent);
      
      protected:
        ROIWidget();
        ~ROIWidget();

      private:
        ROIWidget(const ROIWidget&)= delete;
        void operator=(const ROIWidget&) = delete;
      
      public:
        void SetRepresentation(ROIRepresentation *r) {this->Superclass::SetWidgetRepresentation(reinterpret_cast<vtkWidgetRepresentation*>(r));}

      public:
        void CreateDefaultRepresentation();

      private:
        enum EnWidgetState 
        {
          enStart=0,
          enActive
        };

        // These methods handle events
        static void SelectAction(vtkAbstractWidget*);
        static void EndSelectAction(vtkAbstractWidget*);
        static void TranslateAction(vtkAbstractWidget*);
        static void ScaleAction(vtkAbstractWidget*);
        static void MoveAction(vtkAbstractWidget*);

      private:
        // Manage the state of the widget
        int mWidgetState;
    };
  }
}