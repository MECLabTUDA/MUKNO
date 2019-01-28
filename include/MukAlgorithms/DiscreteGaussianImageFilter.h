#pragma once

#include "ITKWrapper.h"

namespace gris
{
	namespace muk
	{
		/** \brief Class to wrapp itk::DiscreteGaussianImageFilter
		*/
		class MUK_ALGO_API DiscreteGaussianImageFilter : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
		{
		  public:
			  DiscreteGaussianImageFilter();

		  public:
			  static  const char* s_name() { return "DiscreteGaussianImageFilter"; };
			  virtual const char* name()   const { return s_name(); }
		};

	} // namespace muk
} // gris