#include "private/muk.pch"
#include "private/BezierSpiral3DInterpolator.h"
#include "BezierSpline.h"

#include "MukCommon/MukState.h"
#include "MukCommon/MukException.h"

#include <Eigen/Dense>

using namespace Eigen;
using namespace gris::muk;

namespace
{
}

namespace gris
{
namespace muk
{
  /**
  */
  BezierSpiral3DInterpolator::BezierSpiral3DInterpolator()
  {
  }

  /**
  */
  bool BezierSpiral3DInterpolator::interpolatable(const MukState& from, const MukState& to) const
  {
    return true;
  }

  /** \brief

    tangents have to be normalized!
  */
  bool BezierSpiral3DInterpolator::interpolate(const MukState& from, const MukState& to, BezierSpline& output)
  { 
    auto connectingLine = to.coords - from.coords;
    const double inFront         = from.tangent.dot(connectingLine);
    const double sameOrientation = from.tangent.dot(to.tangent);
    const double length = connectingLine.norm();
    
    
    using namespace Eigen;
    Hyperplane<double, 3>       hp    (Vector3d(from.tangent.data()), Vector3d(to.coords.data()));
    ParametrizedLine<double, 3> line  (Vector3d(from.coords.data()), Vector3d(from.tangent.data()));
    auto ipoint = line.intersectionPoint(hp);

    Vec3d binormal  = Vec3d(ipoint.x(), ipoint.y(), ipoint.z()) - to.coords;
    binormal.normalize();
    
    output.sample(BezierSpline::B0) = from.coords;
    output.sample(BezierSpline::E0) = to.coords;
    
    Vec3d connection_b = 0.5 * length * from.tangent;
    output.sample(BezierSpline::B1) = from.coords + 1/3.0 * connection_b;
    output.sample(BezierSpline::B2) = from.coords + 2/3.0 * connection_b;
    
    if (inFront > 0)
    {
      Vec3d connection_e = 0.5 * length * to.tangent;
      output.sample(BezierSpline::E1) = to.coords - 1/3.0 * connection_e;
      output.sample(BezierSpline::E2) = to.coords - 2/3.0 * connection_e;    
      output.sample(BezierSpline::B3) = 0.5 * (output.sample(BezierSpline::B2) + output.sample(BezierSpline::E2));
    }
    else
    { 
    }
    return true;
  }

}
}