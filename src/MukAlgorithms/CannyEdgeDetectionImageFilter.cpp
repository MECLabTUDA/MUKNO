#include "private/muk.pch"
#include "CannyEdgeDetectionImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkCannyEdgeDetectionImageFilter.h>
#include <itkCastImageFilter.h>

namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(CannyEdgeDetectionImageFilter);

		/**

		cast Filter and set properties
		*/
		CannyEdgeDetectionImageFilter::CannyEdgeDetectionImageFilter()
		{
			mDspType = enDisplayImage3D;

      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

			using CastTypeInput = itk::CastImageFilter<ImageInt3D, ImageFloat3D>;
			inputFilter = make_itk<CastTypeInput>();
			using Type = itk::CannyEdgeDetectionImageFilter<ImageFloat3D, ImageFloat3D>;
			cannyEdgeFilter = make_itk<Type>();
			using CastTypeOutput = itk::CastImageFilter<ImageFloat3D, ImageInt3D>;
			outputFilter = make_itk<CastTypeOutput>();
			auto* ptr = dynamic_cast<Type*>(cannyEdgeFilter.GetPointer());

			declareProperty<double>("Variance",
				[=](const double d) { ptr->SetVariance(d); },
				[=]() { return ptr->GetVariance()[0]; });
			declareProperty<MukFloatPixel>("Lower",
				[=](const MukFloatPixel d) { ptr->SetLowerThreshold(d); },
				[=]() { return ptr->GetLowerThreshold(); });
			declareProperty<MukFloatPixel>("Upper",
				[=](const MukFloatPixel d) { ptr->SetUpperThreshold(d); },
				[=]() { return ptr->GetUpperThreshold(); });
		}
	} // namespace muk
} // namespace gris