#pragma once

#include "MukImaging/MukImage.h"

#include <vector>

namespace gris
{
  namespace muk
  {
     void extractRegionsFromLabelImage(std::vector<std::vector<ImageInt3D::IndexType>>& v, const ImageInt3D* pImg);

     void minimize(const ImageInt3D::IndexType& in, ImageInt3D::IndexType& out);
     void maximize(const ImageInt3D::IndexType& in, ImageInt3D::IndexType& out);
  }
}
