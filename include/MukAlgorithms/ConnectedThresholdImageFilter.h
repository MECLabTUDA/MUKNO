#pragma once

#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::ConnectedThresholdImageFilter
    */
    class MUK_ALGO_API ConnectedThresholdImageFilter : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
    {
      public:
        ConnectedThresholdImageFilter();

        static  const char* s_name() { return "ConnectedThresholdImageFilter"; };
        virtual const char* name()   const { return s_name(); }

      protected:
        itk::Index<3u> seed;
    };
  } // namespace muk
} // gris