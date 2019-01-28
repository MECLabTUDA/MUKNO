#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Creates a binary/Segmentation image by thresholding at a lower and/or upper limit.
    */
    class MUK_ALGO_API BinaryThresholdFilter : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
    {
      public:
		    BinaryThresholdFilter();

      public:
        static  const char* s_name()       { return "BinaryThresholdFilter"; };
        virtual const char* name()   const { return s_name(); }
    };

  } // namespace muk
} // gris