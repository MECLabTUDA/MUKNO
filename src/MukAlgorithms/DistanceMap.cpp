#include "private/muk.pch"
#include "DistanceMap.h"
#include "AlgorithmFactory.h"
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"
#include <itkDanielssonDistanceMapImageFilter.h>

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(DistanceMap);

    /**
    */
    DistanceMap::DistanceMap()
    {
      using Type = itk::DanielssonDistanceMapImageFilter<ImageInt3D, ImageInt3D>;
      mpFilter = make_itk<Type>();
      mDspType = enDisplayOverlay3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);
    }
  }
}