#pragma once

#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::MedianImageFilter
    */
    class MUK_ALGO_API MedianSliceImageFilter : public ItkImageToImageWrapper<ImageInt2D, ImageInt2D>
    {
      public:
        MedianSliceImageFilter();

      public:
        static  const char* s_name()       { return "MedianSliceImageFilter"; };
        virtual const char* name()   const { return s_name(); }
    };

  } // namespace muk
} // gris