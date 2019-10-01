#pragma once
#include "opt/Variable.h"

#include "MukCommon/MukVector.h"

namespace gris
{
  namespace bezier
  {
    /** \brief A State in SE3 = R^3 x SO(3)
    */
    template<class T>
    struct Pose
    {
      Pose()      {}
      Pose(const Vec3d& p, const T& rot) : mPosition(p), mOrientation(rot) {}

      const Vec3d&  pos() const {return mPosition ;}
      const T&      rot() const {return mOrientation ;}
      Vec3d&  pos()       {return mPosition ;}
      T&      rot()       {return mOrientation ;}

      Vec3d mPosition;
      T     mOrientation;
    };

    using PoseEuler   = Pose<Vec3d>;

    struct PointFromVariables
    {
      PointFromVariables() {}
      PointFromVariables(opt::Variable x, opt::Variable y, opt::Variable z)
        : mX(x), mY(y), mZ(z)
      {
      }

      Vec3d value(const std::vector<double>& x)
      {
        Vec3d p;
        p.x() = mX.value(x);
        p.y() = mY.value(x);
        p.z() = mZ.value(x);
        return p;
      }

      opt::Variable mX;
      opt::Variable mY;
      opt::Variable mZ;
    };
  }
}
