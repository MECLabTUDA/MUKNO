#include "private/muk.pch"
#include "PrunerDummy.h"

#include "MukCommon/MukException.h"
#include "MukCommon/geometry.h"
#include "MukCommon/OptimizerFactory.h"

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
  REGISTER_OPTIMIZER(PrunerDummy);
  
  /**
  */
  PrunerDummy::PrunerDummy()
  {
  }

  /**
  */
  PrunerDummy::~PrunerDummy()
  {
  }

  /**
  */
  MukPath PrunerDummy::calculate(const MukPath& input) const
  {
    return input;
  }
}
}
