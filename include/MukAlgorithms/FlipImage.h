#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Creates a binary/Segmentation image by thresholding at a lower and/or upper limit.
    */
    class MUK_ALGO_API FlipImage : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
    {
      public:
        FlipImage();

      public:
        static  const char* s_name()       { return "FlipImage"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void   update();
    };

  } // namespace muk
} // gris