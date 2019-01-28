#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief perform  itk::CastImageFilter and returns ImageInt2D
    */
    class MUK_ALGO_API FloatSliceImageToSliceImageFilter : public ItkImageToImageWrapper<ImageFloat2D, ImageInt2D>
    {
      public:
        FloatSliceImageToSliceImageFilter();

      public:
        static  const char* s_name()       { return "FloatSliceImageToSliceImageFilter"; };
        virtual const char* name()   const { return s_name(); }
    };

  } // namespace muk
} // gris