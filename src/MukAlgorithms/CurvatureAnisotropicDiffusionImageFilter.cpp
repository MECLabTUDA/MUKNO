#include "private/muk.pch"
#include "CurvatureAnisotropicDiffusionImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkCurvatureAnisotropicDiffusionImageFilter.h>
#include <itkCastImageFilter.h>

namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(CurvatureAnisotropicDiffusionImageFilter);

		/**

		cast Filter and set properties
		*/
		CurvatureAnisotropicDiffusionImageFilter::CurvatureAnisotropicDiffusionImageFilter()
		{
			mDspType = enDisplayImage3D;

			using CastTypeInput = itk::CastImageFilter<ImageInt3D, ImageFloat3D>;
			inputFilter = make_itk<CastTypeInput>();

			using Type = itk::CurvatureAnisotropicDiffusionImageFilter<ImageFloat3D, ImageFloat3D>;
			diffusionFilter = make_itk<Type>();

			using CastTypeOutput = itk::CastImageFilter<ImageFloat3D, ImageInt3D>;
			outputFilter = make_itk<CastTypeOutput>();

			auto* ptr = dynamic_cast<Type*>(diffusionFilter.GetPointer());

			ptr->SetNumberOfIterations(2);
			ptr->SetTimeStep(0.01125);
			ptr->SetConductanceParameter(3.0);

			declareProperty<itk::IdentifierType>("Iterations",
				[=](const itk::IdentifierType n) { ptr->SetNumberOfIterations(n); },
				[=]() { return ptr->GetNumberOfIterations(); });

			declareProperty<double>("TimeStep",
				[=](const double d) { ptr->SetTimeStep(d); },
				[=]() { return ptr->GetTimeStep(); });

			declareProperty<double>("Conductance",
				[=](const double d) { ptr->SetConductanceParameter(d); },
				[=]() { return ptr->GetConductanceParameter(); });
		}
	} // namespace muk
} // namespace gris