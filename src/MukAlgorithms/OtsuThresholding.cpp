#include "private/muk.pch"
#include "OtsuThresholding.h"
#include "AlgorithmFactory.h"
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"
#include <itkOtsuMultipleThresholdsImageFilter.h>

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(OtsuThresholding);

    /** 

    cast Filter and set properties
    */
    OtsuThresholding::OtsuThresholding()
    {
      using Type = itk::OtsuMultipleThresholdsImageFilter<ImageInt3D, ImageInt3D>;
      mpFilter = make_itk<Type>();
      mDspType = enDisplayOverlay3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

      auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
      declareProperty<int>("NumberOfThresholds",
        [=] (int i) { ptr->SetNumberOfThresholds(i); },
        [=] ()      { return static_cast<int>(ptr->GetNumberOfThresholds()); });
    }
  } // namespace muk
} // namespace gris