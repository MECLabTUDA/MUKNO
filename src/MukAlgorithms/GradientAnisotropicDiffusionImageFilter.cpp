#include "private/muk.pch"
#include "GradientAnisotropicDiffusionImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkGradientAnisotropicDiffusionImageFilter.h>
#include <itkCastImageFilter.h>

namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(GradientAnisotropicDiffusionImageFilter);

		/**
		For parameters see https://itk.org/Doxygen/html/classitk_1_1AnisotropicDiffusionFunction.html
		*/
		GradientAnisotropicDiffusionImageFilter::GradientAnisotropicDiffusionImageFilter()
		{
			mDspType = enDisplayImage3D;

			using Type = itk::GradientAnisotropicDiffusionImageFilter<ImageInt3D, ImageFloat3D>;
			smoothingFilter = make_itk<Type>();

			outputFilter = make_itk<itk::CastImageFilter<ImageFloat3D, ImageInt3D>>();

			auto* ptr = dynamic_cast<Type*>(smoothingFilter.GetPointer());
			ptr->SetNumberOfIterations(1);
			ptr->SetTimeStep(0.01);
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