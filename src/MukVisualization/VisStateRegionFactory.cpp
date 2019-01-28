#include "private/muk.pch"
#include "PolyDataHandler.h"
#include "VisStateRegionFactory.h"
//#include "VisMukStateRegion.h"

// these are the new interactive regions
#include "VisSphereRegion.h"
#include "VisSurfaceRegion.h"
#include "VisPlaneRegion.h"


#include "MukCommon/muk_common.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukStateRegion.h"
#include "MukCommon/PlaneStateRegion.h"
#include "MukCommon/SurfaceStateRegion.h"
#include "MukCommon/StateRegionSingleDirection.h"

#include <vtkPolyData.h>
#include <vtkPoints.h>

namespace gris
{
namespace muk
{
  /** \brief creates the appropriate VisStateRegion for a given IStateRegion

    haven't thought about a convenient architecture yet
  */
  std::shared_ptr<VisStateRegion> VisStateRegionFactory::create(const IStateRegion& region)
  {
    const std::string dummyName = "AnonymousVisRegion";
    const std::string name = region.name();
    if (name == MukStateRegion::s_name())
    {
      auto pObj = std::make_shared<VisSphereRegion>(dummyName);
      const auto& reg = dynamic_cast<const MukStateRegion&>(region);
      pObj->setState(reg.getCenter());
      return pObj;
    }
    else if (name == SurfaceStateRegion::s_name())
    {
      auto pObj = std::make_shared<VisSurfaceRegion>(dummyName);
      const auto& reg = dynamic_cast<const SurfaceStateRegion&>(region);
      pObj->setSurfacePoints (reg.getPoints());
      pObj->setDirectionAncor(reg.getDirectionAncor());
      return pObj;
    }
    else if (name == PlaneStateRegion::s_name())
    {
      auto pObj = std::make_shared<VisPlaneRegion>(dummyName);
      const auto& reg = dynamic_cast<const PlaneStateRegion&>(region);
      pObj->setPlane (reg.getOrigin(), reg.getNormal());
      pObj->setDirectionAncor(reg.getDirection());
      pObj->setBounds(reg.getBoundingPolygon());
      return pObj;
    }
    throw MUK_EXCEPTION("no state region available for this kind of class name", name.c_str());
  }

}
}
