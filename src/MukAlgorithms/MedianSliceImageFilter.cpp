#include "private/muk.pch"
#include "MedianSliceImageFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkMedianImageFilter.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(MedianSliceImageFilter);

  /** 

    cast Filter and set properties
  */
  MedianSliceImageFilter::MedianSliceImageFilter()
  {
    using Type = itk::MedianImageFilter<ImageInt2D, ImageInt2D>;
    mpFilter = make_itk<Type>();
    mDspType = enDisplayImage2D;
    mInputPortTypes.push_back(enImageInt2D);
    mOutputPortTypes.push_back(enImageInt2D);

    auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
    declareProperty<Vec2i>("Radius",
      [=] (const Vec2i& p) { itk::Size<2u> q; q[0] = p.x(); q[1] = p.y(); ptr->SetRadius(q); },
      [=] ()               { const auto r = ptr->GetRadius(); return Vec2i(r[0], r[1]); });
  }
} // namespace muk
} // namespace gris