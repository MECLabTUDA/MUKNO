#include "private/muk.pch"
#include "VisAbstractObject.h"

#include "vtkActor.h"

namespace gris
{
namespace muk
{
  /**
  */
  VisAbstractObject::VisAbstractObject(const std::string& name)
    : VisualObject()
    , mName(name)
    , mpTransform(vtkSmartPointer<vtkTransform>::New())
  {
  }

  /**
  */
  VisAbstractObject::~VisAbstractObject()
  {
  }

  /**
  */
  void VisAbstractObject::setName(const std::string& name)
  {
    mName = name;
  }

  /**
  * this uses the Actor(s) UserTransform.
  *
  */
  void VisAbstractObject::setTransform(const vtkSmartPointer<vtkTransform> transform)
  {
    mpTransform->DeepCopy(transform);
//    mpTransform->Scale(mScale, mScale, mScale);
  }

  /**
  */
  vtkSmartPointer<vtkTransform> VisAbstractObject::getTransform() const
  {
    return mpTransform;
  }

  void VisAbstractObject::assignTransformToActor()
  {
    mpActor->SetUserTransform(mpTransform);
    std::for_each(mpActors.begin(), mpActors.end(),
      [this](vtkSmartPointer<vtkActor>& actor)
    {
      actor->SetUserTransform(mpTransform);
    }
    );
  }

  /**
  Usefullness circumstantial, mostly for for multiple actors

  void VisAbstractObject::setScale(const double scale)
  {
    if (mpTransform != nullptr)
    { // rescale to new scale
      const double relativescale = scale / mScale;
      mpTransform->Scale(relativescale, relativescale, relativescale);
    }
    mScale = scale;
  }

  double VisAbstractObject::getScale() const
  {
    return mScale;
  }
  */
}
}