#include "private/muk.pch"
#include "ConfidenceConnectedImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkConfidenceConnectedImageFilter.h>

namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(ConfidenceConnectedImageFilter);

		/**

		cast Filter and set properties
		*/
		ConfidenceConnectedImageFilter::ConfidenceConnectedImageFilter()
		{
			using Type = itk::ConfidenceConnectedImageFilter<ImageInt3D, ImageInt3D>;
			mpFilter = make_itk<Type>();
			mDspType = enDisplayImage3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

			seed = itk::Index<3U>();
			seed[0] = 0;
			seed[1] = 0;
			seed[2] = 0;

			auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());

			// ptr->SetConnectivity(itk::ConnectedThresholdImageFilter<ImageInt3D, ImageInt3D>::ConnectivityEnumType::FullConnectivity);
			ptr->SetMultiplier(2.5);
			ptr->SetNumberOfIterations(2);
			ptr->SetSeed(seed);

			declareProperty<Vec3i>("Seed",
				[=](const Vec3i& p) { seed[0] = p.x(); seed[1] = p.y(); seed[2] = p.z(); ptr->SetSeed(seed); },
				[=]() { const auto r = seed; return Vec3i(r[0], r[1], r[2]); });
			declareProperty<gris::muk::MukPixel>("Replace",
				[=](const gris::muk::MukPixel p) { ptr->SetReplaceValue(p); },
				[=]() { return ptr->GetReplaceValue(); });

			declareProperty<double>("Multiplier",
				[=](const double d) { ptr->SetMultiplier(d); },
				[=]() { return ptr->GetMultiplier(); });
			declareProperty<unsigned int>("Iterations",
				[=](const unsigned int n) { ptr->SetNumberOfIterations(n); },
				[=]() { return ptr->GetNumberOfIterations(); });
		}
	} // namespace muk
} // namespace gris