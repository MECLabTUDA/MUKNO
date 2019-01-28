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
		class MUK_ALGO_API SliceImageBinaryNot : public ItkImageToImageWrapper<ImageInt2D, ImageInt2D>
		{
		public:
			SliceImageBinaryNot();
		public:
			static  const char* s_name() { return "SliceImageBinaryNot"; };
			virtual const char* name()   const { return s_name(); }
		};

	} // namespace muk
} // gris

