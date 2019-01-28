#include "private/muk.pch"
#include "ThresholdFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkThresholdImageFilter.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(ThresholdFilter);

  /**
  */
  ThresholdFilter::ThresholdFilter()
  {
    using Type = itk::ThresholdImageFilter<ImageInt3D>;
    mpFilter = make_itk<Type>();
    mDspType = enDisplayImage3D;
    mInputPortTypes.push_back(enImageInt3D);
    mOutputPortTypes.push_back(enImageInt3D);

    auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
    declareProperty<MukPixel>("LowerThreshold",
      [=] (const MukPixel& p)   { ptr->SetLower(p); },
      [=] ()                    { return ptr->GetLower(); });
    declareProperty<MukPixel>("UpperThreshold",
      [=] (const MukPixel& p)  { ptr->SetUpper(p); },
      [=] ()                   { return ptr->GetUpper(); });
  }
}
}