#include "private/muk.pch"
#include "BinaryThresholdFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkBinaryThresholdImageFilter.h>

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(BinaryThresholdFilter);

    /**
    */
	BinaryThresholdFilter::BinaryThresholdFilter()
    {
      using Type = itk::BinaryThresholdImageFilter<ImageInt3D, ImageInt3D>;
      mpFilter = make_itk<Type>();
      mDspType = enDisplayOverlay3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

      auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
      ptr->SetInsideValue(1);
      declareProperty<MukPixel>("LowerThreshold",
        [=] (const MukPixel& p)   { ptr->SetLowerThreshold(p); },
        [=] ()                    { return ptr->GetLowerThreshold(); });
      declareProperty<MukPixel>("UpperThreshold",
        [=] (const MukPixel& p)  { ptr->SetUpperThreshold(p); },
        [=] ()                   { return ptr->GetUpperThreshold(); });
      declareProperty<MukPixel>("Label",
        [=] (const MukPixel& p)  { ptr->SetInsideValue(p); },
        [=] ()                   { return ptr->GetInsideValue(); });
    }
  }
}