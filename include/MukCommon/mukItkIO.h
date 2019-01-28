#pragma once

#include "private/muk.pch"
#include "MukCommon\muk_common_api.h"
#include "itkSmartPointer.h"
#include "itkImage.h"
#include <string>

namespace gris
{
	namespace muk
	{
		MUK_COMMON_API itk::Image<unsigned short, 3>::Pointer loadCtData(const std::string& filename);

		MUK_COMMON_API void separateMixedSegmentations(const std::string& filename);		
	}
}
