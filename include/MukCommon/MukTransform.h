#pragma once

#include "muk_common_api.h"

// VTK
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

#include <chrono>

namespace gris
{
namespace muk
{

  class MUK_COMMON_API MukTransform
  {
    public:
      typedef vtkSmartPointer<vtkTransform> vtkPointer;
      typedef std::chrono::steady_clock     clock;

    public:
      MukTransform();
      MukTransform(vtkTransform* ptransform, const std::string& transformname, MukTransform::clock::time_point time = MukTransform::clock::now());
      MukTransform(const MukTransform& transform);
      ~MukTransform();

      MukTransform& operator=(const MukTransform& rhs);

      bool operator==(const MukTransform& other);
      bool operator<(const MukTransform& other);
      bool operator>(const MukTransform& other);

      // name-related
      bool is(const std::string& name) const;
      std::string getName() const;
      void setName(const std::string& name);
      operator std::string() const { return getName(); }

      // Acquisition-time related
      clock::time_point getAcquistionTime() const;
      void setAcquisitionTime(const clock::time_point& time);

    public:
      operator vtkPointer() const;
      void setTransform(vtkTransform* transform);
      vtkTransform* getTransform() const;
     
  private:
      vtkSmartPointer<vtkTransform> mpTransform;
      std::string                   mTransformName;
      clock::time_point             mAcqTime;
  };
}
}

namespace std
{
  MUK_COMMON_API std::ostream& operator<< (std::ostream& os, const gris::muk::MukTransform& obj);
//  MUK_COMMON_API std::istream& operator>> (std::istream& is, gris::muk::MukTransform& obj);
}

