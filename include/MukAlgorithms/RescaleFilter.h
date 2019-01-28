#pragma once

#include "ITKWrapper.h"

#include <itkRescaleIntensityImageFilter.h>

namespace gris
{
	namespace muk
	{
		/** \brief Creates a binary/Segmentation image by thresholding at a lower and/or upper limit.
		*/
		class MUK_ALGO_API RescaleFilter : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
		{
		  public:
			  RescaleFilter();

		  public:
			  static  const char* s_name() { return "RescaleFilter"; };
			  virtual const char* name()   const { return s_name(); }
		};
	} // namespace muk
} // grsi