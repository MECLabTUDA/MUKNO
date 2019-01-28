#pragma once

#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::ConnectedThresholdImageFilter
    */
    class MUK_ALGO_API ConnectedThresholdSliceImageFilter : public ItkImageToImageWrapper<ImageInt2D, ImageInt2D>
    {
      public:
        ConnectedThresholdSliceImageFilter();

        static  const char* s_name() { return "ConnectedThresholdSliceImageFilter"; };
        virtual const char* name()   const { return s_name(); }

      protected:
        itk::Index<2u> seed;
    };
  } // namespace muk
} // gris