#include "private/muk.pch"
#include "VisVector.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include <vtkArrowSource.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>

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
  VisVector::VisVector(const std::string& name)
    : VisScaleableObject(name)
    , mpArrow(make_vtk<vtkArrowSource>())
  {
    mpMapper->SetInputConnection(mpArrow->GetOutputPort());
  }

  /**
  */
  VisVector::~VisVector()
  {
  }

  /**
  */
  void VisVector::update()
  {
    //arrowSource->SetTipLength();
    //arrowSource->SetTipRadius();
    //arrowSource->SetShaftRadius();
  }

  void VisVector::setVector(const Vec3d & from, const Vec3d & normal)
  {
    mpActor->SetOrigin(from.data());
    auto n = normal;
    n.normalize();
    Vec3d a = n;
    a.cross(n);
    Vec3d b = n;
    b.cross(a);
    auto matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    matrix->SetElement(0, 0, n.x());
    matrix->SetElement(1, 0, n.y());
    matrix->SetElement(2, 0, n.z());
    matrix->SetElement(0, 1, a.x());
    matrix->SetElement(0, 1, a.y());
    matrix->SetElement(1, 1, a.z());
    matrix->SetElement(2, 2, b.x());
    matrix->SetElement(1, 2, b.y());
    matrix->SetElement(2, 2, b.z());
    double orientation[3];
    vtkTransform::GetOrientation(orientation, matrix);
    mpActor->SetOrientation(orientation);
  }

  vtkSmartPointer<vtkArrowSource> VisVector::getArrowSource()
  {
    return mpArrow;
  }

}
}