#include "private/muk.pch"
#include "muk_imaging_tools.h"

namespace gris
{
namespace muk
{
  /**
  */
  void allocateFromSrc(const ImageInt3D* src, ImageInt3D* target, bool fill, MukPixel val)
  {
    target->SetRegions(src->GetBufferedRegion());
    target->SetOrigin(src->GetOrigin());
    target->SetDirection(src->GetDirection());
    target->SetSpacing(src->GetSpacing());
    target->Allocate();
    if (fill)
      target->FillBuffer(val);
  }

  void allocateFromSrc(const ImageInt3D& src, ImageInt3D& target, bool fill, MukPixel val)
  {
    allocateFromSrc(&src, &target, fill, val);
  }
}
}