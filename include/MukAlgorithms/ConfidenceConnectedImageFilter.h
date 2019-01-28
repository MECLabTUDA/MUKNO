#pragma once

#include "ITKWrapper.h"

namespace gris
{
	namespace muk
	{
		/** \brief Class to wrapp itk::ConfidenceConnectedImageFilter
		*/
		class MUK_ALGO_API ConfidenceConnectedImageFilter : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
		{
		  public:
			  ConfidenceConnectedImageFilter();

			  static  const char* s_name() { return "ConfidenceConnectedImageFilter"; };
			  virtual const char* name()   const { return s_name(); }

		  protected:
			  itk::Index<3u> seed;
		};

	} // namespace muk
} // gris