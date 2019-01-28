#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Creates a binary/Segmentation image by thresholding at a lower and/or upper limit.
    */
    class MUK_ALGO_API BinaryThresholdSliceFilter : public ItkImageToImageWrapper<ImageInt2D, ImageInt2D>
    {
      public:
		    BinaryThresholdSliceFilter();

      public:
        static  const char* s_name()       { return "BinaryThresholdSliceFilter"; };
        virtual const char* name()   const { return s_name(); }
    };

  } // namespace muk
} // gris