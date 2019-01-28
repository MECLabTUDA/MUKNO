#pragma once

#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::MedianImageFilter
    */
    class MUK_ALGO_API MedianImageFilter : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
    {
      public:
        MedianImageFilter();

      public:
        static  const char* s_name()       { return "MedianImageFilter"; };
        virtual const char* name()   const { return s_name(); }
    };

  } // namespace muk
} // gris