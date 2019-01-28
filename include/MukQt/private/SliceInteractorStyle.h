#pragma once

#include <vtkInteractorStyleImage.h>
#include <SliceRenderGroup.h>

class vtkImageData;

namespace gris
{
  namespace muk
  {
    class SliceInteractorStyle : public vtkInteractorStyleImage
    {
      public:
        static SliceInteractorStyle* New();
        vtkTypeMacro(SliceInteractorStyle, vtkInteractorStyleImage);

      public:
        virtual void OnMouseMove();
        virtual void OnMiddleButtonDown();
        /*virtual void OnLeftButtonDown();
        virtual void OnLeftButtonUp();
        
        virtual void OnMiddleButtonUp();
        virtual void OnRightButtonDown();
        virtual void OnRightButtonUp();*/
        virtual void OnMouseWheelForward();
        virtual void OnMouseWheelBackward();

      public:
        void setImage(vtkImageData* pImage) { mpImage = pImage; }
        void setMedicalOrientation(EnMedicalOrientation en) { mMedOrient = en; }
        void zoomToFit();

      private:
        void slice(int stepsize);

      private:
        int mScrollStep = 1;
        int mScrollStepLarge = 5;
        EnMedicalOrientation mMedOrient;
        vtkImageData* mpImage = nullptr;
    };
  }
}