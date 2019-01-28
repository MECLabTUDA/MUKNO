#pragma once
#include "ITKWrapper.h"

#include "MukCommon/muk_common.h"

#include <itkBinaryNotImageFilter.h>

namespace gris
{
	namespace muk
	{
		/** \brief Class to wrapp itk::BinaryNotImageFilter
		*/
		class MUK_ALGO_API BinaryNot : public ItkImageToImageWrapper<ImageInt3D, ImageInt3D>
		{
		  public:
			  BinaryNot();

		  public:
			  static  const char* s_name() { return "BinaryNot"; };
			  virtual const char* name()   const { return s_name(); }
		};

	} // namespace muk
} // gris

