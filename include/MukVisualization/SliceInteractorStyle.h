#pragma once
#include "muk_visualization_api.h"

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCommand.h>
class vtkImageData;
class vtkImageSliceMapper;

#include <functional>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API SliceInteractorStyle : public vtkInteractorStyleTrackballCamera
    {
      public:
        static SliceInteractorStyle *New();
        vtkTypeMacro(SliceInteractorStyle, vtkInteractorStyleTrackballCamera);

      public:
        SliceInteractorStyle();

      public:
        virtual void OnEnter();

        //@{
        /**
        * Event bindings controlling the effects of pressing mouse buttons
        * or moving the mouse.
        */
        virtual void OnMouseMove();
        virtual void OnMouseWheelForward();
        virtual void OnMouseWheelBackward();
        virtual void OnLeftButtonDown();
        virtual void OnLeftButtonUp();
        virtual void OnMiddleButtonDown();
        virtual void OnMiddleButtonUp();
        virtual void OnRightButtonDown();
        virtual void OnRightButtonUp();

        virtual void OnChar();

        virtual void Pick();
        
        // Interaction mode entry points used internally.
        virtual void StartPick();
        virtual void EndPick();

      public:
        enum EnCustomEvent
        {
          enSliceChangedEvent = vtkCommand::UserEvent + 1,
        };

        using CallbackPixelValue = std::function<void(int, int, int, double)>;

      public:
        void slice(int count);
        void zoomToFit();
        void turnUpsideDown();
        void setCallbackPixelValue(CallbackPixelValue f) { mCallback = f; mHasCallback = true;  }

        void setOrientationToX();
        void setOrientationToY();
        void setOrientationToZ();

      private:
          enum EnOrientationType
          {
            enXView,
            enYView,
            enZView,
          };

      private:
          vtkImageData*         getImage();
          vtkImageSliceMapper*  getMapper();
          EnOrientationType getOrientationType() const;

        private:
          void setImageOrientation(const double leftToRight[3], const double viewUp[3]);

      private:
        double mXViewRightVector[3];
        double mXViewUpVector[3];
        double mYViewRightVector[3];
        double mYViewUpVector[3];
        double mZViewRightVector[3];
        double mZViewUpVector[3];

        CallbackPixelValue mCallback;
        bool mHasCallback = false;
    };
  }
}