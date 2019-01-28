#include "private/muk.pch"
#include "LaplacianImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkLaplacianImageFilter.h>
#include <itkCastImageFilter.h>

namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(LaplacianImageFilter);

		/**

		cast Filter and set properties
		*/
		LaplacianImageFilter::LaplacianImageFilter()
		{
			mDspType = enDisplayImage3D;

			using CastTypeInput = itk::CastImageFilter<ImageInt3D, ImageFloat3D>;
			inputFilter = make_itk<CastTypeInput>();

			using Type = itk::LaplacianImageFilter<ImageFloat3D, ImageFloat3D>;
			laplacianFilter = make_itk<Type>();

			using CastTypeOutput = itk::CastImageFilter<ImageFloat3D, ImageInt3D>;
			outputFilter = make_itk<CastTypeOutput>();

			auto* ptr = dynamic_cast<Type*>(laplacianFilter.GetPointer());

			declareProperty<bool>("Physical spacing",
				[=](const bool b) { ptr->SetUseImageSpacing(b); },
				[=]() { return ptr->GetUseImageSpacing(); });
		}
	} // namespace muk
} // namespace gris
