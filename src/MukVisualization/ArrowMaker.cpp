#include "private/muk.pch"
#include "private/ArrowMaker.h"

#include "MukCommon/muk_common.h"

#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>

namespace
{
  const double arbitrary[3] = { vtkMath::Random(-10, 10), vtkMath::Random(-10, 10), vtkMath::Random(-10, 10) };
}

namespace gris
{
namespace muk
{
  /**
  */
  void createArrow(const MukState& state, vtkTransform* pArrowTransform)
  {
    // Generate start and end point
    double startPoint[3] = { state.coords.x(), state.coords.y(), state.coords.z() };
    double endPoint[3]   = { state.coords.x() + state.tangent.x(), state.coords.y() + state.tangent.y(), state.coords.z() + state.tangent.z() };
    // Compute a basis
    double normalizedX[3];
    double normalizedY[3];
    double normalizedZ[3];
    // The X axis is a vector from start to end
    vtkMath::Subtract(endPoint, startPoint, normalizedX);
    double length = vtkMath::Norm(normalizedX);
    vtkMath::Normalize(normalizedX);
    // The Z axis is an arbitrary vector cross X
    vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
    vtkMath::Normalize(normalizedZ);
    // The Y axis is Z cross X
    vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
    auto matrix = make_vtk<vtkMatrix4x4>();
    // Create the direction cosine matrix
    matrix->Identity();
    for (unsigned int i = 0; i < 3; i++)
    {
      matrix->SetElement(i, 0, normalizedX[i]);
      matrix->SetElement(i, 1, normalizedY[i]);
      matrix->SetElement(i, 2, normalizedZ[i]);
    }
    // Apply the transforms
    pArrowTransform->Identity();
    pArrowTransform->Translate(startPoint);
    pArrowTransform->Concatenate(matrix);
    pArrowTransform->Scale(length, length, length);
    pArrowTransform->Modified();
  }
}
}