#pragma once

#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::DiscreteGaussianImageFilter
    */
    class MUK_ALGO_API DiscreteGaussian : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
    {
      public:
        DiscreteGaussian();

      public:
        static  const char* s_name()       { return "DiscreteGaussian"; };
        virtual const char* name()   const { return s_name(); }
    };
  } // namespace muk
} // gris#pragma once
