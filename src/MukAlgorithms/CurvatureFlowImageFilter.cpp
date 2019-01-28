#include "private/muk.pch"
#include "CurvatureFlowImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkCurvatureFlowImageFilter.h>
#include <itkCastImageFilter.h>


namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(CurvatureFlowImageFilter);

		/**
		cast Filter and set properties
		*/
		CurvatureFlowImageFilter::CurvatureFlowImageFilter()
		{
			mDspType = enDisplayImage3D;

			using Type = itk::CurvatureFlowImageFilter<ImageInt3D, ImageFloat3D>;
			curvatureFilter = make_itk<Type>();

			using CastTypeOutput = itk::CastImageFilter<ImageFloat3D, ImageInt3D>;
			outputFilter = make_itk<CastTypeOutput>();

			auto* ptr = dynamic_cast<Type*>(curvatureFilter.GetPointer());
			ptr->SetNumberOfIterations(1);
			ptr->SetTimeStep(0.125);

			declareProperty<itk::IdentifierType>("Iterations",
				[=](const itk::IdentifierType n) { ptr->SetNumberOfIterations(n); },
				[=]() { return ptr->GetNumberOfIterations(); });

			declareProperty<double>("TimeStep",
				[=](const double d) { ptr->SetTimeStep(d); },
				[=]() { return ptr->GetTimeStep(); });
		}
	} // namespace muk
} // namespace gris