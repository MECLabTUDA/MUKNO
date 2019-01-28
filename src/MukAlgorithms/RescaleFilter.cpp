#include "private/muk.pch"
#include "RescaleFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(RescaleFilter);

		/**
		*/
		RescaleFilter::RescaleFilter() 
    {
			using Type = itk::RescaleIntensityImageFilter<ImageInt3D, ImageInt3D>;
			mpFilter = make_itk<Type>();
			mDspType = enDisplayOverlay3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

			auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
			ptr->SetOutputMinimum(0);
			ptr->SetOutputMaximum(4000);

			declareProperty<MukPixel>("Minimum",
				[=](const MukPixel& p) { ptr->SetOutputMinimum(p); },
				[=]() { return ptr->GetOutputMinimum(); });
			declareProperty<MukPixel>("Maximum",
				[=](const MukPixel& p) { ptr->SetOutputMaximum(p); },
				[=]() { return ptr->GetOutputMaximum(); });
		}
	}
}