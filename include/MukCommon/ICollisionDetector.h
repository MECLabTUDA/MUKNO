#pragma once

#include "muk_common_api.h"
#include "MukVector.h"

#include <memory>
#include <string>

template<class T> class vtkSmartPointer;
class vtkPolyData;

namespace gris
{
  namespace muk
  {

    /**
    */
    class MUK_COMMON_API ICollisionDetector
    {
      public:
        ICollisionDetector() : mOutOfDate(true) {}
        virtual ~ICollisionDetector() {}
        
      public:
        virtual void addObstacle    (const std::string& key, vtkSmartPointer<vtkPolyData>& obj) = 0;
        virtual void removeObstacle (const std::string& key) = 0;
        virtual void setActive      (const std::string& key, bool on) = 0;
        virtual bool isActive       (const std::string& key) const = 0;
        virtual bool hasKey         (const std::string& key) const = 0;
        virtual std::vector<std::string> getKeys        () const = 0;
        
      public:
        virtual bool nearestNeighbor(const Vec3d& p, Vec3d& q, double maxDist=0) const = 0;
        virtual bool hasNeighbors(const Vec3d& p, double maxDist)                const = 0;
        virtual void rebuild() const {}
        virtual void clear() = 0;
        
        bool isOutOfDate() const { return mOutOfDate; }

      protected:
        mutable bool mOutOfDate;
    };

  }
}