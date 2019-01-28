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
    class MUK_ALGO_API ImageReader : public ItkImageSourceWrapper<ImageInt3D>
    {
      public:
		    ImageReader();

      public:
        static const char* s_name() { return "ImageReader"; };
        virtual const char* name()  const { return s_name(); }
    };
  }
}