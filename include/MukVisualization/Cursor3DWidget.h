#pragma once
#include "muk_visualization_api.h"

class vtkImageData;
#include <vtkAbstractWidget.h>

namespace gris
{
  namespace muk
  {
    class Cursor3DRepresentation;

    /**
    */
    class MUK_VIS_API Cursor3DWidget : public vtkAbstractWidget
    {
      public:
        static Cursor3DWidget *New();
        vtkTypeMacro(Cursor3DWidget,vtkAbstractWidget);

      protected:
        Cursor3DWidget();
        ~Cursor3DWidget();

      private:
        Cursor3DWidget(const Cursor3DWidget&)= delete;
        void operator=(const Cursor3DWidget&) = delete;

      public:
        void SetRepresentation(Cursor3DRepresentation *r) {this->Superclass::SetWidgetRepresentation(reinterpret_cast<vtkWidgetRepresentation*>(r));}

      public:
        void CreateDefaultRepresentation();
        void setPosition(double pos[3]);
        void setImage(vtkImageData* pImage);

      private:
        enum EnWidgetState 
        {
          enStart=0,
          enActive
        };

      private:
        // These methods handle events
        static void SelectAction(vtkAbstractWidget*);
        static void EndSelectAction(vtkAbstractWidget*);
        static void TranslateAction(vtkAbstractWidget*);
        static void MoveAction(vtkAbstractWidget*);

      private:
        // Manage the state of the widget
        int mWidgetState;
    };
  }
}
