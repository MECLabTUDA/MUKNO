#include "private/muk.pch"
#include "Skeletonization2.h"
#include "AlgorithmFactory.h"

#include <private/SkeletonizationFilter.h>

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(Skeletonization2);

    /**
    */
    Skeletonization2::Skeletonization2()
    {
      using Type = itk::SkeletonizationFilter<ImageInt3D, ImageInt3D>;
      mpFilter = make_itk<Type>();
      mDspType = enDisplayOverlay3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);
    }
  }
}