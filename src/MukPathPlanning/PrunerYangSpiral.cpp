#include "private/muk.pch"
#include "PrunerYangSpiral.h"

#include "MukCommon/MukException.h"
#include "MukCommon/geometry.h"
#include "MukCommon/PrunerFactory.h"

#include <Eigen/Dense>

namespace
{
  using gris::Vec3d;
  using namespace gris::muk;

  double angle(const Vec3d& lhs, const Vec3d& rhs)
  {
    using namespace Eigen;
    Vector3d l(lhs.data());
    Vector3d r(rhs.data());
    l.normalize();
    r.normalize();
    return acos(l.dot(r));
  }
}

namespace gris
{
  namespace muk
  {
    REGISTER_PRUNER(PrunerYangSpiral);

    /**
    */
    struct PrunerYangSpiral::Impl
    {
      Impl() {}
      ~Impl() {}

      bool line_intersects(const Vec3d& point, const Vec3d& line, ICollisionDetector* pColl, double maxDist);

      ICollisionDetector* coll;
    };

    /**
    */
    PrunerYangSpiral::PrunerYangSpiral()
      : mp(std::make_unique<Impl>())
    {
    }

    /**
    */
    PrunerYangSpiral::~PrunerYangSpiral()
    {
    }

    /**
    */
    void PrunerYangSpiral::clone(const IPathPruner* pOther)
    {
      setKappa(pOther->getKappa());
      setMaxDistance(pOther->getMaxDistance());
      setMaxStepSize(pOther->getMaxStepSize());
    }

    /**
    */
    MukPath PrunerYangSpiral::calculate(const MukPath& input)
    {
      if (nullptr == mpCollision.get())
      {
        throw MUK_EXCEPTION_SIMPLE("Collission-detector has not been set");
      }      
      MukPath output;
      output.setRadius(input.getRadius());
      auto& dest      = output.getPath();
      const auto& src = input.getPath();    
      dest.reserve(src.size());

      if (src.size() < 6) // 1 point in the middle needed at least
      {
        std::copy(src.begin(), src.end(), back_inserter(dest));
      }
      else
      {
        std::copy(src.begin(), src.begin() + 2, back_inserter(dest));
        typedef std::vector<MukState>::iterator Iter;
        typedef std::vector<MukState>::const_iterator CIter;

        Iter gparent = dest.begin();     // grandparent of running variable state
        Iter parent  = dest.begin() + 1; 
        CIter state   = src.begin() + 2;
        CIter child, gchild; // grandchild of state

        Sphere3D sphere;
        auto lowerCurvature = [&] (const Vec3d& p1, const Vec3d& p2, const Vec3d& p3)
        {
          fitSphere(p1, p2, p3, sphere);
          return mKappa > 1.0 / sphere.getRadius();
        };

        while (state < src.end()-2)
        {
          gchild = src.end()-1;
          child  = gchild-1;
          bool shortcutFound = false;
          Vec3d S0 = 0.5*(gparent->coords + parent->coords);
          const double d1 = (parent->coords - gparent->coords).norm();
          while (child != state && !shortcutFound)
          {          
            Vec3d S3 = 0.5*(gchild->coords + child->coords);
            Vec3d t  = child->coords-parent->coords;          
            t.normalize();
            const double d2 = (child->coords - gchild->coords).norm();
            Vec3d S1 = parent->coords + d1 * t;
            Vec3d S2 = child->coords - d2 * t;
            // see chapter 4 (picture): extraneous node pruning
            if  (lowerCurvature(S0, parent->coords, S1)
              && lowerCurvature(S2, child->coords, S3)
              && ! mp->line_intersects(S0, S1-S0, mpCollision.get(), mMaxDist)
              && ! mp->line_intersects(S1, S2-S1, mpCollision.get(), mMaxDist)
              && ! mp->line_intersects(S2, S3-S2, mpCollision.get(), mMaxDist) )
            {
              shortcutFound = true;            
            }
            else
            {
              --child;
              --gchild;
            }
          }
          // if "found", state can be omitted -> add child
          if (shortcutFound)
          {          
            dest.push_back(*child);
            state = child+1;
          }
          else
          {
            dest.push_back(*state);
            ++state;
          }
          ++parent;
          ++gparent;
        }
        // could already be inserted, if shourcut was found in the last step
        if (dest.back() != *(src.end()-2))
          dest.push_back(*(src.end()-2));
        dest.push_back(src.back());
      }
      return output;
    }

    /**
    */
    bool PrunerYangSpiral::Impl::line_intersects(const Vec3d& point, const Vec3d& line, ICollisionDetector* pColl, double maxDist)
    {
      const double length = line.norm();
      const double stepSize = 0.5;
      const size_t steps = static_cast<size_t>( std::ceil(length / stepSize) );
      const Vec3d  ray   = stepSize*1.0/length * line;
      const double MAX_DIST = 1.0;
      Vec3d runningPoint = point;
      for (size_t i(0); i<steps; ++i)
      {
        if (pColl->hasNeighbors(runningPoint, maxDist))
          return true;
        runningPoint += ray;
      }
      return false;
    }

  }
}
