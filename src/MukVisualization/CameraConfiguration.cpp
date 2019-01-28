#include "private/muk.pch"
#include "CameraConfiguration.h"

namespace gris
{
namespace muk
{
  /**
  */
  CameraConfiguration::CameraConfiguration()
    : position(0,0,1)
    , focalPoint(0,0,0)
    , viewUp (0,1,0)
  {
    // this is tde vtk Default
    // a combination of these three can be invalid resulting in a rendering bug (e.g. (0,0,0), (0,0,0), (0,0,0) is invalid)
    declareProperty<Vec3d>("Position",
      [&] (const Vec3d& p) { setPosition(p); },
      [&] ()               { return getPosition(); });
    declareProperty<Vec3d>("FocalPoint",
      [&] (const Vec3d& p) { setFocalPoint(p); },
      [&] ()               { return getFocalPoint(); });
    declareProperty<Vec3d>("ViewUp",
      [&] (const Vec3d& p) { setViewUp(p); },
      [&] ()               { return getViewUp(); });
  }
}
}

namespace std
{
  std::ostream& operator<<(std::ostream& os, const gris::muk::CameraConfiguration& config)
  {
    return os;
  }

  std::istream& operator>>(std::istream& is, gris::muk::CameraConfiguration& config)
  {
    return is;
  }
}