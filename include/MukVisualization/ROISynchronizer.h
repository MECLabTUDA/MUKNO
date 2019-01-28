#pragma once
#include "ROIWidget.h"
#include "muk_visualization_api.h"

#include <vtkCallbackCommand.h>

#include <vector>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API ROISynchronizer : public vtkCallbackCommand
    {  
      public:
        static ROISynchronizer *New();
        vtkTypeMacro(ROISynchronizer,vtkCallbackCommand);

      public:
        ROISynchronizer();

      public:
        virtual void Execute(vtkObject* caller, unsigned long, void*);

      public:
        void initialize(ROIWidget* w1, ROIWidget* w2, ROIWidget* w3);


      private:
        std::vector<ROIWidget*> mpWidgets;
    };
  }
}