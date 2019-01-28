#include "private/muk.pch"
#include "MukTransform.h"

#include <string>

namespace gris
{
namespace muk
{

  MukTransform::MukTransform()
    : mpTransform(vtkSmartPointer<vtkTransform>::New())
    , mTransformName("Unknown")
    , mAcqTime(clock::now())
  {
    // initialize mMatrix with Identity
    mpTransform->Identity();
  }
  
  MukTransform::MukTransform(vtkTransform * ptransform, const std::string& transformname, MukTransform::clock::time_point time)
    : mTransformName(transformname)
    , mpTransform(vtkSmartPointer<vtkTransform>::New())
    , mAcqTime(time)
  {
    mpTransform->DeepCopy(ptransform);
  }

  MukTransform::MukTransform(const MukTransform & other)
    : mTransformName(other.mTransformName)
    , mpTransform(vtkSmartPointer<vtkTransform>::New())
    , mAcqTime(other.mAcqTime)
  {
    mpTransform->DeepCopy(other.mpTransform);
  }

  MukTransform::~MukTransform()
  {
  }

  bool MukTransform::is(const std::string & name) const
  {
    return mTransformName == name;
  }

  std::string MukTransform::getName() const
  {
    return mTransformName;
  }

  void MukTransform::setName(const std::string & name)
  {
    mTransformName = name;
  }

  MukTransform::clock::time_point MukTransform::getAcquistionTime() const
  {
    return mAcqTime;
  }

  void MukTransform::setAcquisitionTime(const clock::time_point & time)
  {
    mAcqTime = time;
  }

  MukTransform & MukTransform::operator=(const MukTransform & rhs)
  {
    mAcqTime = rhs.mAcqTime;
    mTransformName = rhs.mTransformName;
    mpTransform->DeepCopy(rhs.getTransform());
    return *this;
  }

  MukTransform::operator vtkPointer() const
  {
    return mpTransform;
  }

  void MukTransform::setTransform(vtkTransform * transform)
  {
    mpTransform = vtkPointer(transform);
  }

  vtkTransform* MukTransform::getTransform() const
  {
    return mpTransform;
  }

  bool MukTransform::operator==(const MukTransform & other)
  {
    return mTransformName == other.mTransformName;
  }

  bool MukTransform::operator<(const MukTransform & other)
  {
    return mTransformName < other.mTransformName;
  }

  bool MukTransform::operator>(const MukTransform & other)
  {
    return mTransformName > other.mTransformName;
  }

}
}

std::ostream & std::operator<<(std::ostream & os, const gris::muk::MukTransform & obj)
{
  vtkSmartPointer<vtkTransform> pTransform = obj;
  os << obj.getName() << ": ";
  pTransform->PrintSelf(os, vtkIndent());
  return os;
}
