#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief perform  itk::DiscreteGaussianImageFilter and returns MukFloatImage
    */
    class MUK_ALGO_API GaussianSliceFilter : public ItkImageToImageWrapper<ImageInt2D, ImageFloat2D>
    {
      public:
        GaussianSliceFilter();

      public:
        static  const char* s_name()       { return "GaussianSliceFilter"; };
        virtual const char* name()   const { return s_name(); }
    };

  } // namespace muk
} // gris