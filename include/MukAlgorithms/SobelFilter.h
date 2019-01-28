#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::SobelEdgeDetectionImageFilter
    */
    class MUK_ALGO_API SobelFilter : public ItkImageToImageWrapper<ImageInt3D, ImageFloat3D>
    {
      public:
        SobelFilter();

      public:
        static  const char* s_name()       { return "SobelFilter"; };
        virtual const char* name()   const { return s_name(); }
    };

  } // namespace muk
} // gris