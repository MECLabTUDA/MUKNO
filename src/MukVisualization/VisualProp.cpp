#include "private/muk.pch"
#include "VisualProp.h"
#include "muk_colors.h"

#include "MukCommon/MukException.h"
#include "MukCommon/overload_deduction.h"

#include <vtkRenderer.h>

#include <boost/format.hpp>

using namespace gris::muk;

namespace gris
{
namespace muk
{
  /**
  */
  VisualProp::VisualProp(vtkSmartPointer<vtkProp3D> prop)
    : mpActor( prop )
    , mpRenderer( nullptr )
  {
  }
    
  /**
  */
  VisualProp::~VisualProp()
  {
    if (mpRenderer)
      mpRenderer->RemoveActor(mpActor);
  }

  /**
  */
  void VisualProp::setRenderer(vtkRenderer* pRenderer)
  {
    if (mpRenderer)
      mpRenderer->RemoveActor(mpActor);
    mpRenderer = pRenderer;
    if (pRenderer)
      pRenderer->AddActor(mpActor);
  }

  /**
  */
  void VisualProp::setColors(const std::vector<Vec3d>& colors)
  {
  }

  void VisualProp::setTransform(const vtkSmartPointer<vtkTransform> transform)
  {
    mpActor->SetUserTransform(transform);
  }

  vtkSmartPointer<vtkTransform> VisualProp::getTransform()
  {
    auto transform = vtkSmartPointer<vtkTransform>(vtkTransform::SafeDownCast(mpActor->GetUserTransform()));
    if (transform == nullptr)
    {
      transform = vtkSmartPointer<vtkTransform>::New();
      transform->SetMatrix(mpActor->GetUserTransform()->GetMatrix());
    }
    return transform;
  }

}
}