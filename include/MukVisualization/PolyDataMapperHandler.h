#pragma once

#include "muk_visualization_api.h"

#include "MukCommon/MukVector.h"

#include <vector>

class vtkPolyDataMapper;

namespace gris
{
  namespace muk
  {

    struct MUK_VIS_API PolyDataMapperHandler
    {
      public:
      static void addTube(vtkPolyDataMapper* mapper, const double radius, const size_t numberOfSlides);
      static void setColor(vtkPolyDataMapper* mapper, const std::vector<Vec3d>& colors);
      static void setColor(vtkPolyDataMapper* mapper, const double* color);
      static void clearTopology(vtkPolyDataMapper* mapper);
    };


  }
}
