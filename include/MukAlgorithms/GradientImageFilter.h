#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::SobelEdgeDetectionImageFilter
    */
    class MUK_ALGO_API GradientImageFilter : public ItkImageToImageWrapper<ImageFloat3D, GradientImage3D>
    {
      public:
        GradientImageFilter();

      public:
        static  const char* s_name()       { return "GradientImageFilter"; };
        virtual const char* name()   const { return s_name(); }
    };
  } // namespace muk
} // gris