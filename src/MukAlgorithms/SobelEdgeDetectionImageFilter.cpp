#include "private/muk.pch"
#include "SobelEdgeDetectionImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkSobelEdgeDetectionImageFilter.h>
#include <itkCastImageFilter.h>

namespace gris
{
namespace muk
{
	REGISTER_ALGORITHM(SobelEdgeDetectionImageFilter);

	/**

	cast Filter and set properties
	*/
	SobelEdgeDetectionImageFilter::SobelEdgeDetectionImageFilter()
	{
		mDspType = enDisplayImage3D;

		using Type = itk::SobelEdgeDetectionImageFilter<ImageInt3D, ImageFloat3D>;
		inputFilter = make_itk<Type>();

		using CastType = itk::CastImageFilter<ImageFloat3D, ImageInt3D>;
		outputFilter = make_itk<CastType>();
	}
} // namespace muk
} // namespace gris