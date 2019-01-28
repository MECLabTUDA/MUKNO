#pragma once
#include "ITKWrapper.h"

#include "MukCommon/muk_common.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::ImageReader
	    Data source that reads image data from a single file.
    */
    class MUK_ALGO_API SliceImageReader : public ItkImageSourceWrapper<ImageInt2D>
    {
      public:
		    SliceImageReader();

      public:
        static const char* s_name() { return "SliceImageReader"; };
        virtual const char* name()  const { return s_name(); }
    };
  }
}