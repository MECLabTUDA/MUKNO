#pragma once
#include "muk_algorithms_api.h"

#include "MukImaging/MukImage.h"

#include <itkMesh.h>
#include <itkDefaultDynamicMeshTraits.h>

#include <vtkPolyData.h>

namespace gris
{
  namespace muk
  {
    using MukMeshPrecision = float;
    using MeshTrait        = itk::DefaultDynamicMeshTraits<MukMeshPrecision, 3, 3, MukMeshPrecision, MukMeshPrecision>;
    using TriangleMesh     = itk::Mesh<MukMeshPrecision, Dim3D, MeshTrait>;

    MUK_ALGO_API void polyDataToItkMesh   (const vtkPolyData& poly, TriangleMesh& mesh);
    MUK_ALGO_API void itkMeshToVtkPolyData(const TriangleMesh& mesh, vtkPolyData& poly);
  }
}