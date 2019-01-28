#pragma once
#include "muk_imaging_api.h"
#include "MukImage.h"

namespace gris
{
  namespace muk
  {
    MUK_IMG_API ImageInt3D::Pointer loadDicom(const std::string& directory);
    MUK_IMG_API ImageInt3D::Pointer loadMhd(const std::string& filename);
  }
}
