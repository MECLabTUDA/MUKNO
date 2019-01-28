#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Creates a grayscale image by thresholding at a lower and/or upper limit.
    */
    class MUK_ALGO_API RescaleIntensity : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
    {
      public:
        RescaleIntensity();

      public:
        static  const char* s_name()       { return "RescaleIntensity"; };
        virtual const char* name()   const { return s_name(); }
    };
  } // namespace muk
} // gris