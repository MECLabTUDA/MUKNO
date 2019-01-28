#pragma once

#include "muk_common_api.h"
#include "ICollisionDetector.h"

namespace gris
{
  namespace muk
  {

    class MUK_COMMON_API CollisionDetectorKdTree : public ICollisionDetector
    {
      public:
        CollisionDetectorKdTree();
        virtual ~CollisionDetectorKdTree();

      public:
        virtual void addObstacle(const std::string& key, vtkSmartPointer<vtkPolyData>& obj);
        virtual void removeObstacle (const std::string& key);
        virtual void setActive      (const std::string& key, bool b);
        virtual bool isActive       (const std::string& key) const;
        virtual bool hasKey         (const std::string& key) const;
        virtual std::vector<std::string> getKeys        () const;

      public:
        virtual bool nearestNeighbor(const Vec3d& p, Vec3d& nn, double maxDist=0) const;
        virtual bool hasNeighbors   (const Vec3d& p, double maxDist)              const;

        virtual void rebuild() const;
        virtual void clear();

      private:        
        struct Impl;
        std::unique_ptr<Impl> mp;
    };

  }
}