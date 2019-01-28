#include "private/muk.pch"
#include "VisMultiPortSphereRegion.h"
#include "MultiPortSphereRegionRepresentation.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MultiPortStateRegion.h"
#include "MukCommon/MukProblemDefinition.h"

#include <vtkImageData.h>
#include <vtkImplicitPlaneWidget2.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>


namespace gris
{
  namespace muk
  {
    /**
    */
    VisMultiPortSphereRegion::VisMultiPortSphereRegion(const std::string& name)
      : VisStateRegion(name)
      , mpImpPlaneWidget(make_vtk<vtkImplicitPlaneWidget2>())
      , mpSphereRep(make_vtk<MultiPortSphereRegionRepresentation>())
      , mResolution(20)
    {
      const auto llc = Vec3d(-1.0);
      const auto urc = Vec3d( 1.0);
      double bounds[6] = { llc.x(), urc.x(), llc.y(), urc.y(), llc.z(), urc.z()}; // initial position
      mpSphereRep->SetPlaceFactor(1.0);
      mpSphereRep->PlaceWidget(bounds);
      mpImpPlaneWidget->SetRepresentation(mpSphereRep);

      declareProperty<Vec3i>("ActivePorts",
        [&] (const Vec3i& p)  { mpSphereRep->setPortActivity(p); },
        [&] ()                { return mpSphereRep->getPortActivity(); });
      declareProperty<unsigned int>("SampleResolution",
        [&] (unsigned int p)  { mResolution = p; },
        [&] ()                { return mResolution; });
    }

    /**
    */
    VisMultiPortSphereRegion::~VisMultiPortSphereRegion()
    {
    }

    /**
    */
    vtkImplicitPlaneWidget2* VisMultiPortSphereRegion::widget()
    {
      return mpImpPlaneWidget.Get(); 
    }

    /**
    */
    MultiPortSphereRegionRepresentation* VisMultiPortSphereRegion::representation()
    {
      return mpSphereRep.Get(); 
    }

    /**
    */
    void VisMultiPortSphereRegion::setRenderer(vtkRenderer* pRenderer)
    {
      VisualObject::setRenderer(pRenderer);
      mpImpPlaneWidget->SetInteractor(pRenderer->GetRenderWindow()->GetInteractor());
    }

    /**
    */
    const MukState& VisMultiPortSphereRegion::getCenter() const
    {
      mCenter = MukState(Vec3d(mpSphereRep->GetOrigin()), Vec3d(1,0,0));
      return mCenter;
    }

    /**
    */
    std::unique_ptr<IStateRegion> VisMultiPortSphereRegion::asStateRegion(bool asStart) const
    {
      auto res = std::make_unique<MultiPortStateRegion>();
      const auto& ports = mpSphereRep->getPortActivity();
      // collect states
      std::vector<MukState> states;
      for (size_t i(0); i<3; ++i)
      {
        if ( ! ports[i])
          continue;
        states.push_back(mpSphereRep->getState(i));
        if (!asStart)
          states.back().tangent *= -1;
      }
      if (states.empty())
        throw MUK_EXCEPTION_SIMPLE("No States available to create state region");
      res->setNumberOfStates(states.size());
      for (size_t i(0); i<states.size(); ++i)
        res->setState(i, states[i]);
      // set common parameters
      res->setPhi(mpSphereRep->getAngleThreshold());
      res->setRadius(mpSphereRep->getSphereRadius());
      res->setResolution(mResolution);
      return std::move(res);
    }

    /**
    */
    void VisMultiPortSphereRegion::setFromProblemDefinition(const MukProblemDefinition& obj)
    {
      mpSphereRep->setAngleThreshold(obj.getGoalAngleThreshold());
      mpSphereRep->setSphereRadius(obj.getGoalThreshold());
      mpSphereRep->setPathRadius(obj.getRadius());
    }

    /**
    */
    bool VisMultiPortSphereRegion::isMovable() const
    {
      return true;
    }

    /** \brief
    */
    void VisMultiPortSphereRegion::swap(VisMultiPortSphereRegion* other)
    {
      std::swap(mpImpPlaneWidget, other->mpImpPlaneWidget);
      std::swap(mpSphereRep, other->mpSphereRep);
      std::swap(mResolution,other->mResolution);
    }
  }
}