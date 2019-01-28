#pragma once

#include <vtkInteractorStyleTrackballCamera.h>

namespace gris
{
  namespace muk
  {
    class VtkWindowInteraction : public vtkInteractorStyleTrackballCamera
    {
      public:
        static VtkWindowInteraction* New();
        vtkTypeMacro(VtkWindowInteraction, vtkInteractorStyleTrackballCamera);

      public:
        VtkWindowInteraction();

      public:        
        virtual void OnLeftButtonDown();
        virtual void OnMouseMove();
        virtual void OnRightButtonDown();
        virtual void OnRightButtonUp();
        virtual void OnMouseWheelForward();
        virtual void OnMouseWheelBackward();
        virtual void OnKeyPress();

        void setRenderer(vtkRenderer*);
        void setLogics(std::shared_ptr<AppLogics> pObj)         { mpLogics = pObj; }
        Vec3d getSelectedPoint();

        void setStartRegion();
        void setGoalRegion();
        void setStartPoint();
        void setGoalPoint();
        void setWaypoint();
        void createSphere();
        void showClosestPoint();
        void showClosestPointToPath();
        void setDefaultFocus();

      private:
    };
  }
}
