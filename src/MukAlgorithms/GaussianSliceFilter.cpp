#include "private/muk.pch"
#include "GaussianSliceFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include "itkDiscreteGaussianImageFilter.h"
namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(GaussianSliceFilter);

    /**
    */
    GaussianSliceFilter::GaussianSliceFilter()
    {
      using Type = itk::DiscreteGaussianImageFilter<ImageInt2D, ImageFloat2D>;
      mpFilter = make_itk<Type>();
      mDspType = enDisplayOverlay2D;
      mInputPortTypes.push_back(enImageInt2D);
      mOutputPortTypes.push_back(enImageInt2D);

      auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
      ptr->SetVariance(4.0);
    }
  }
}