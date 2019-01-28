#pragma once
#include "muk_qt_api.h"

#include <vtkCallbackCommand.h>
class vtkRenderWindowInteractor;
class vtkInteractorStyle;

#include <vector>

namespace gris
{
  namespace muk
  {
    class Cursor3DWidget;
    class SliceRenderGroup;
    class SliceWidget;
    class ROIRepresentation;
    class ROIWidget;

    /**
    */
    class MUK_QT_API Cursor3DSynchronizer : public vtkCallbackCommand
    {  
    public:
      static Cursor3DSynchronizer *New();
      vtkTypeMacro(Cursor3DSynchronizer,vtkCallbackCommand);

    public:
      Cursor3DSynchronizer();

    public:
      virtual void Execute(vtkObject* caller, unsigned long, void*);

    public:
      void initialize(Cursor3DWidget* w1, Cursor3DWidget* w2, Cursor3DWidget* w3);
      void initialize(SliceRenderGroup& g1, SliceRenderGroup& g2, SliceRenderGroup& g3);
      void initialize(ROIWidget* g1, ROIWidget* g2, ROIWidget* g3);
      void initialize(SliceWidget* g1, SliceWidget* g2, SliceWidget* g3);

    private:
      std::vector<Cursor3DWidget*>      mpCursorWidgets;
      std::vector<vtkInteractorStyle*>  mpInteractors;
      std::vector<SliceRenderGroup*>    mpGroups;
      std::vector<SliceWidget*>         mpSliceWidgets;
      std::vector<ROIRepresentation*>   mpROIs;
    };
  }
}