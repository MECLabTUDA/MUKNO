#pragma once

#include "muk_imaging_api.h"

#include "MukCommon/MukVector.h"

#include <itkImage.h>
#include <itkSmartPointer.h>


namespace gris
{
  namespace muk
  {
    const size_t Dim3D = 3u;
    const size_t Dim2D = 2u;

    using MukPixel       = short;
    using MukFloatPixel  = float;

    using ImageInt3D      = itk::Image<MukPixel, Dim3D>;
    using ImageInt2D      = itk::Image<MukPixel, Dim2D>;
    using ImageFloat2D    = itk::Image<MukFloatPixel, Dim2D>;
    using ImageFloat3D    = itk::Image<MukFloatPixel, Dim3D>;
    using GradientImage3D = itk::Image<itk::CovariantVector<MukFloatPixel,Dim3D>,Dim3D>;

    using ItkImageIndex = ImageInt3D::IndexType;

    using ImageInt3DIndex = Vec3i;
    using ItkPoint      = itk::Point<double, Dim3D>;
  }
}
