#pragma once

#include "MukCommon/MukVector.h"
#include "MukCommon/muk_common.h"

#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSphereSource.h>
#include <vtkLineSource.h>
#include <vtkPolyData.h>
#include <vtkIntersectionPolyDataFilter.h>

namespace gris
{
  namespace muk
  {
    class SphereInteraction;
    /** \brief Defines interaction within the vtkDirectionWindow
    */
    class DirectionInteractionStyle : public vtkInteractorStyleTrackballCamera
    {
      public:
        static DirectionInteractionStyle* New();
        vtkTypeMacro(DirectionInteractionStyle, vtkInteractorStyleTrackballCamera);

      public:
        DirectionInteractionStyle();

      public:        
        virtual void OnLeftButtonDown();
        virtual void OnLeftButtonUp();
        virtual void OnMouseMove();

      public:
        void setInteractor(SphereInteraction* pInter) { mpInteractor = pInter; }

      private:
        void compute();

      private:
        bool mousePressed;
        SphereInteraction* mpInteractor;
    };
  }
}
