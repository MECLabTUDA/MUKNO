#include "private/muk.pch"
#include "FloatSliceImageToSliceImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include "itkCastImageFilter.h"
namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(FloatSliceImageToSliceImageFilter);

    /**
    */
    FloatSliceImageToSliceImageFilter::FloatSliceImageToSliceImageFilter()
    {
      using Type = itk::CastImageFilter<ImageFloat2D, ImageInt2D>;
      mpFilter = make_itk<Type>();
      mDspType = enDisplayOverlay3D;
      mInputPortTypes.push_back(enImageInt2D);
      mOutputPortTypes.push_back(enImageInt2D);
    }
  }
}