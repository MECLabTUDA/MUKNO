#pragma once

#include "private/IMukInteraction.h"

#include <vtkInteractorStyleTrackballCamera.h>

#include <memory>

namespace gris
{
  namespace muk
  {
    class SurfaceCutter : public IMukInteraction, public vtkInteractorStyleTrackballCamera
    {
      public:
        static SurfaceCutter* New();
        vtkTypeMacro(SurfaceCutter, vtkInteractorStyleTrackballCamera);

      public:
        SurfaceCutter();

      public:
        virtual void OnLeftButtonDown();
        virtual void OnMouseMove();
        virtual void OnRightButtonDown();
        virtual void OnRightButtonUp();
        virtual void OnMouseWheelForward();
        virtual void OnMouseWheelBackward();
        virtual void OnKeyPress();

        virtual void initialize();

      private:
        void accept();
        void discard();

        private:
        void computeRegion();
        void finishInteraction();
        void pick();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
