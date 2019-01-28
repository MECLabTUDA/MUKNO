#pragma once

#include "muk_algorithms_api.h"

#include "MukImaging/MukImage.h"

class vtkPolyData;

namespace gris
{
  namespace muk
  {
    MUK_ALGO_API ImageInt3D::Pointer meshToBinaryImage(ImageInt3D* referenceImage, vtkPolyData* mesh, MukPixel label = 1u);
  }
}

