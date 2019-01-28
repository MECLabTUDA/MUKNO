#include "private/muk.pch"
#include "ConnectedThresholdImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkConnectedThresholdImageFilter.h>

namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(ConnectedThresholdImageFilter);

		/**

		cast Filter and set properties
		*/
		ConnectedThresholdImageFilter::ConnectedThresholdImageFilter()
		{
			using Type = itk::ConnectedThresholdImageFilter<ImageInt3D, ImageInt3D>;
			mpFilter = make_itk<Type>();
			mDspType = enDisplayImage3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);
			seed = itk::Index<3U>();
			seed[0] = 0;
			seed[1] = 0;
			seed[2] = 0;

			auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
			
			ptr->SetConnectivity(itk::ConnectedThresholdImageFilter<ImageInt3D, ImageInt3D>::ConnectivityEnumType::FullConnectivity);
			ptr->SetLower(210);
			ptr->SetUpper(240);
			ptr->SetSeed(seed);
			
			declareProperty<Vec3i>("Seed",
				[=](const Vec3i& p) { seed[0] = p.x(); seed[1] = p.y(); seed[2] = p.z(); ptr->SetSeed(seed); },
				[=]() { const auto r = seed; return Vec3i(r[0], r[1], r[2]); });
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