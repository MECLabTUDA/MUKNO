#include "private/muk.pch"
#include "DiscreteGaussianImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkDiscreteGaussianImageFilter.h>

namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(DiscreteGaussianImageFilter);

		/**

		cast Filter and set properties
		*/
		DiscreteGaussianImageFilter::DiscreteGaussianImageFilter()
		{
			using Type = itk::DiscreteGaussianImageFilter<ImageInt3D, ImageInt3D>;
			mpFilter = make_itk<Type>();
			mDspType = enDisplayImage3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

			auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());

			declareProperty<double>("Variance",
				[=](const double d) { ptr->SetVariance(d); },
				[=]() { return ptr->GetVariance()[0]; });
		}
	} // namespace muk
} // namespace gris