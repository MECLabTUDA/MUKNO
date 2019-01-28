#include "private/muk.pch"
#include "MedianImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkMedianImageFilter.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(MedianImageFilter);

  /** 

    cast Filter and set properties
  */
  MedianImageFilter::MedianImageFilter()
  {
    using Type = itk::MedianImageFilter<ImageInt3D, ImageInt3D>;
    mpFilter = make_itk<Type>();
    mDspType = enDisplayImage3D;
    mInputPortTypes.push_back(enImageInt3D);
    mOutputPortTypes.push_back(enImageInt3D);

    auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
    declareProperty<Vec3i>("Radius",
      [=] (const Vec3i& p) { itk::Size<3u> q; q[0] = p.x(); q[1] = p.y(); q[2] = p.z(); ptr->SetRadius(q); },
      [=] () { const auto r = ptr->GetRadius(); return Vec3i(r[0], r[1], r[2]); });

  }
} // namespace muk
} // namespace gris