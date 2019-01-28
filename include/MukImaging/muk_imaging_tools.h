#pragma once

#include "muk_imaging_api.h"
#include "MukImage.h"

namespace gris
{
  namespace muk
  {
    template<typename T>
    typename T::Pointer make_itk()  // c++14 auto make_itk()s
    {
      return T::New();
    }

    MUK_IMG_API void allocateFromSrc(const ImageInt3D* src, ImageInt3D* target, bool fill = true, MukPixel val = 0);
    MUK_IMG_API void allocateFromSrc(const ImageInt3D& src, ImageInt3D& target, bool fill = true, MukPixel val = 0);
  }
}