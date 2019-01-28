#include "private/muk.pch"
#include "SobelFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkSobelEdgeDetectionImageFilter.h>

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(SobelFilter);

    /** 

    cast Filter and set properties
    */
    SobelFilter::SobelFilter()
    {
      using Type = itk::SobelEdgeDetectionImageFilter<ImageInt3D, ImageFloat3D>;
      mpFilter = make_itk<Type>();
      mDspType = enDisplayImage3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageFloat3D);
    }
  } // namespace muk
} // namespace gris