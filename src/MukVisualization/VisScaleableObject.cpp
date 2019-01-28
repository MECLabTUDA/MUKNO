#include "private/muk.pch"
#include "VisScaleableObject.h"

#include "vtkActor.h"

namespace gris
{
namespace muk
{
  /**
  */
  VisScaleableObject::VisScaleableObject(const std::string& name)
    : VisAbstractObject(name)
  {
  }

  /**
  */
  VisScaleableObject::~VisScaleableObject()
  {
  }

  void VisScaleableObject::update()
  {
    update();
  }

  void VisScaleableObject::setScale(const float scale)
  {
    mpActor->SetScale(scale);
    std::for_each(mpActors.begin(), mpActors.end(), [&scale](DeclVtk(vtkActor) a) { a->SetScale(scale); });
  }

  float VisScaleableObject::getScale() const
  {
    if (mpActor) return *mpActor->GetScale();
    if (!mpActors.empty()) return *mpActors[0]->GetScale();
    return 0;
  }

  void VisScaleableObject::declareProperties()
  {
    declareProperty<float>("Scale",
      [&](float d) { this->setScale(d); },
      [&]() { return this->getScale(); });
  }
}
}