#include "private/muk.pch"
#include "Skeletonization.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkBinaryThinningImageFilter.h>

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(Skeletonization);

    /**
    */
    Skeletonization::Skeletonization()
    {
      using Type = itk::BinaryThinningImageFilter<ImageInt3D, ImageInt3D>;
      mpFilter = make_itk<Type>();
      mDspType = enDisplayOverlay3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);
    }
  }
}