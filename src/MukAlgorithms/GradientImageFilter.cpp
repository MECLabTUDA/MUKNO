#include "private/muk.pch"
#include "GradientImageFilter.h"
#include "AlgorithmFactory.h"
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"
#include <itkGradientRecursiveGaussianImageFilter.h>

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(GradientImageFilter);

    /** 

    cast Filter and set properties
    */
    GradientImageFilter::GradientImageFilter()
    {
      using Type = itk::GradientRecursiveGaussianImageFilter<ImageFloat3D, GradientImage3D>;
      mpFilter = make_itk<Type>();
      mDspType = enDisplayImage3D;
      mInputPortTypes.push_back(enImageFloat3D);
      mOutputPortTypes.push_back(enGradientImage3D);
      //auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
      auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
      declareProperty<double>("Sigma",
        [=] (double val) { ptr->SetSigma(val); },
        [=] ()           { return  ptr->GetSigma(); });
    }
  } // namespace muk
} // namespace gris