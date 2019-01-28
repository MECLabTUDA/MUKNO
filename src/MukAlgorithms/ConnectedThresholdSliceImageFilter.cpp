#include "private/muk.pch"
#include "ConnectedThresholdSliceImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkConnectedThresholdImageFilter.h>

namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(ConnectedThresholdSliceImageFilter);

		/**

		cast Filter and set properties
		*/
		ConnectedThresholdSliceImageFilter::ConnectedThresholdSliceImageFilter()
		{
			using Type = itk::ConnectedThresholdImageFilter<ImageInt2D, ImageInt2D>;
			mpFilter = make_itk<Type>();
			
      mDspType = enDisplayOverlay2D;
      mInputPortTypes.push_back(enImageInt2D);
      mOutputPortTypes.push_back(enImageInt2D);

			seed = itk::Index<2U>();
			seed[0] = 0;
			seed[1] = 0;

			auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
			
			ptr->SetConnectivity(itk::ConnectedThresholdImageFilter<ImageInt2D, ImageInt2D>::ConnectivityEnumType::FullConnectivity);
			ptr->SetLower(200);
			ptr->SetUpper(200);
			ptr->SetSeed(seed);
      ptr->SetReplaceValue(0);
			
			declareProperty<Vec2i>("Seed",
				[=](const Vec2i& p) { seed[0] = p.x(); seed[1] = p.y(); ptr->SetSeed(seed); },
				[=]() { const auto r = seed; return Vec2i(r[0], r[1]); });

			declareProperty<gris::muk::MukPixel>("Replace",
				[=](const gris::muk::MukPixel p) { ptr->SetReplaceValue(p); },
				[=]() { return ptr->GetReplaceValue(); });

			declareProperty<gris::muk::MukPixel>("Lower",
				[=](const gris::muk::MukPixel p) { ptr->SetLower(p); },
				[=]() { return ptr->GetLower(); });
			declareProperty<gris::muk::MukPixel>("Upper",
				[=](const gris::muk::MukPixel p) { ptr->SetUpper(p); },
				[=]() { return ptr->GetUpper(); });
		}
	} // namespace muk
} // namespace gris