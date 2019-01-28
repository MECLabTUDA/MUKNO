#pragma once

#include <vtkButtonWidget.h>
#include <vtkCommand.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkTexturedButtonRepresentation2D.h>

#include <functional>

class vtkInteractorObserver;
class vtkRenderWindowInteractor;
class vtkRenderer;
class vtkInteractorStyleDrawPolygon;
class vtkActor;
class vtkPolyDataMapper;
class vtkPolyData;

namespace gris
{
  namespace muk
  {
    /**
    */
    struct SurfaceRegionRepresentationCallback : public vtkCommand
    {
      public:
        SurfaceRegionRepresentationCallback();
        static SurfaceRegionRepresentationCallback* New() { return new SurfaceRegionRepresentationCallback(); }

      public:
        void Execute(vtkObject* caller, unsigned long ev, void*) override;

      private:
        void executeStateChangedEvent(vtkObject* caller);
        void executeSelectionChangedEvent(vtkObject* caller);
        
        using SelectedPointsCallback = std::function<void(vtkSmartPointer<vtkPoints>)>;

      public:
        vtkInteractorObserver* mCache;
        vtkRenderWindowInteractor* mInteractor;
        vtkRenderer* mRenderer;
        vtkSmartPointer<vtkInteractorStyleDrawPolygon> drawer;
        vtkActor* mPickedActor;
        vtkButtonWidget* mButton;
        SelectedPointsCallback mSetPointsCallback;
    };

  }
}