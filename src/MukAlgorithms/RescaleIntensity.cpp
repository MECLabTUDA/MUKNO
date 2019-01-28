#include "private/muk.pch"
#include "RescaleIntensity.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkRescaleIntensityImageFilter.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(RescaleIntensity);

  /**
  */
  RescaleIntensity::RescaleIntensity()
  {
    using Type = itk::RescaleIntensityImageFilter<ImageInt3D>;
    mpFilter = make_itk<Type>();
    mDspType = enDisplayOverlay3D;
    mInputPortTypes.push_back(enImageInt3D);
    mOutputPortTypes.push_back(enImageInt3D);

    auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
    declareProperty<MukPixel>("MinimumIntensity",
      [=] (const MukPixel& p)   { ptr->SetOutputMinimum(p); },
      [=] ()                    { return ptr->GetOutputMinimum(); });
    declareProperty<MukPixel>("MaximumIntensity",
      [=] (const MukPixel& p)  { ptr->SetOutputMaximum(p); },
      [=] ()                   { return ptr->GetOutputMaximum(); });
  }
}
}