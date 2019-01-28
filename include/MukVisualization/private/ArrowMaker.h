#pragma once

#include "MukCommon/MukState.h"

class vtkTransform;

namespace gris
{
  namespace muk
  {
    /**
    */
    void createArrow(const MukState& state, vtkTransform* pArrowTransform);
  }
}
