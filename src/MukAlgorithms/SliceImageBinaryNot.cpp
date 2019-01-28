#include "private/muk.pch"
#include "SliceImageBinaryNot.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"


namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(SliceImageBinaryNot);
		/**
		*/
		SliceImageBinaryNot::SliceImageBinaryNot() {
			using Type = itk::BinaryNotImageFilter<ImageInt2D>;
			mpFilter = make_itk<Type>();
			mDspType = enSegmentation;
			auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
			ptr->SetForegroundValue(1);
		}
	}
}