#include "private/muk.pch"
#include "FlipImage.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkFlipImageFilter.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(FlipImage);

  /**
  */
  FlipImage::FlipImage()
  {
    using Type = itk::FlipImageFilter<ImageInt3D>;
    mpFilter = make_itk<Type>();
    mDspType = enDisplayImage3D;
    mInputPortTypes.push_back(enImageInt3D);
    mOutputPortTypes.push_back(enImageInt3D);

    auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());

    auto flipAxis = [=] (int index, bool flip)
    {
      auto  axis = ptr->GetFlipAxes();
      axis[index] = flip;
      ptr->SetFlipAxes(axis);
    };

    declareProperty<bool>("FlipX",
      [=] (bool b) { flipAxis(0, b); },
      [=] () { return ptr->GetFlipAxes()[0]; });
    declareProperty<bool>("FlipY",
      [=] (bool b) { flipAxis(1, b); },
      [=] () { return ptr->GetFlipAxes()[1]; });
    declareProperty<bool>("FlipZ",
      [=] (bool b) { flipAxis(2, b); },
      [=] () { return ptr->GetFlipAxes()[2]; });
  }
    
  /**
  */
  void FlipImage::update()
  {
    auto origin = mpFilter->GetInput()->GetOrigin();
    auto direction = mpFilter->GetInput()->GetDirection();
    mpFilter->UpdateLargestPossibleRegion();
    mpFilter->GetOutput()->SetOrigin(origin);
    mpFilter->GetOutput()->SetDirection(direction);
  }
}
}