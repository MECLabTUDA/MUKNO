#pragma once

#include "BaseController.h"

#include <vtkInteractorStyleTrackballCamera.h>

class vtkRenderer;
class vtkCommand;

namespace gris
{
namespace muk
{
  struct AppModels;
  struct AppControllers;

  /**
  */
  class IMukInteraction : public BaseController, public vtkInteractorStyleTrackballCamera
  {
    public:
      static IMukInteraction* New();
      vtkTypeMacro(IMukInteraction, vtkInteractorStyleTrackballCamera);

    public:
      virtual ~IMukInteraction() = default;

    public:
      void setRenderer(vtkRenderer* pRenderer) { mpRenderer = pRenderer; }
	    void addObserver(vtkCommand* pObserver, int objectID) { mObservers[objectID] = pObserver; };

      virtual void finalize() {}

    protected:
      vtkRenderer*      mpRenderer;
	    std::vector<vtkCommand*> mObservers;
  };
}
}
