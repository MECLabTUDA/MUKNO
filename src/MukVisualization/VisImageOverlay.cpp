#include "private/muk.pch"
#include "VisImageOverlay.h"

#include "private/ImageOverlayCallback.h"

#include "MukCommon/muk_common.h"

#include <vtkImageData.h>
#include <vtkImplicitPlaneWidget2.h>
#include <vtkImplicitPlaneRepresentation.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

namespace
{
  using namespace gris;

  Vec3d toWorldCoordinates(const Vec3d& origin, const Vec3d& spacing, const Vec3i& voxel)
  {
    return Vec3d(
      origin.x() + voxel.x()*spacing.x(),
      origin.y() + voxel.y()*spacing.y(),
      origin.z() + voxel.z()*spacing.z()
    );
  }
  Vec3i toVoxelCoordinates(const Vec3d& origin, const Vec3d& spacing, const Vec3d& point)
  {
    return Vec3i(
      (point.x() - origin.x()) / spacing.x(),
      (point.y() - origin.y()) / spacing.y(),
      (point.z() - origin.z()) / spacing.z()
    );
  }
}

namespace gris
{
  namespace muk
  {
    /**
    */
    VisImageOverlay::VisImageOverlay(const std::string& name)
      : VisAbstractObject(name)
      , mpImpPlaneWidget(make_vtk<vtkImplicitPlaneWidget2>())
      , mpPlaneRep(make_vtk<vtkImplicitPlaneRepresentation>())
      , mpCallback(make_vtk<ImageOverlayCallback>())
    {
      mPosition = { 0,0,0 };
      mExtent = { 2,2,2 };

      declareProperty<Vec3d>("Extent"
        , [&](const Vec3d& p) { mExtent = p; setPosition(getPosition()); }
      , [&]() { return mExtent; });

      declareProperty<bool>("Interactor"
        , [=](const bool& t) { mpPlaneRep->SetVisibility(t? 1 : 0); mpImpPlaneWidget->SetProcessEvents(t? 1 : 0); }
      , [=]() { return mpPlaneRep->GetVisibility() == 1 ? true : false; });

      declareProperty<bool>("Fixed interaction"
        , std::bind(&VisImageOverlay::setFixedInteraction, this, std::placeholders::_1)
        , [=]() { return mFixedInteraction; });

      declareProperty<bool>("Orthoradial Slices"
        , [=](const bool& t) {mOrthSlices = t; setNormal(mNormal); }
      , [=]() {setNormal(mNormal); return mOrthSlices; });

    }

    /**
    */
    VisImageOverlay::~VisImageOverlay()
    {
      mpImpPlaneWidget->SetInteractor(nullptr);
      mpImpPlaneWidget->RemoveAllObservers();
      mpPlaneRep->RemoveAllObservers();
    }

    /**
    */
    void VisImageOverlay::setRenderer(vtkRenderer* pRenderer)
    {
      VisualObject::setRenderer(pRenderer);
      mpImpPlaneWidget->SetInteractor(pRenderer->GetRenderWindow()->GetInteractor());
      double bounds[6] = { 0, 1, 0, 1, 0, 1 }; // initial position
      mpPlaneRep->PlaceWidget(bounds);
      mpPlaneRep->SetPlaceFactor(1.0);
      mpImpPlaneWidget->SetRepresentation(mpPlaneRep);
      mpImpPlaneWidget->SetProcessEvents(0);    // Disables any interaction with the Widget
      mpCallback->setRenderer(pRenderer);
      mpImpPlaneWidget->On();
      mpCallback->setRepresentation(mpPlaneRep);
      mpImpPlaneWidget->AddObserver("InteractionEvent", mpCallback);
      mpCallback->setVisibility(true);
      mpPlaneRep->SetVisibility(false);
    }

    /**
    */
    void VisImageOverlay::setOpacity(double d)
    {
      VisAbstractObject::setOpacity(d);
      mpCallback->setOpacity(d);
    }

    /**
    */
    void VisImageOverlay::setVisibility(bool visible)
    {
      VisAbstractObject::setVisibility(visible);
      mpCallback->setVisibility(visible);
    }

    /**
    */
    void VisImageOverlay::setFixedInteraction(const bool fixed)
    {
      mFixedInteraction = fixed;
      LOG_LINE << "fixed: " << fixed;
    }

    /**
    */
    void VisImageOverlay::setImage(vtkImageData* pImage)
    {
      mpCallback->setImage(pImage);
    }

    /**
    */
    void VisImageOverlay::setPosition(const Vec3d& p)
    {
      Vec3d shiftedPos = p;
      if (mFixedInteraction)
      {
        mFixedPositionShift = Vec3d(mpPlaneRep->GetOrigin()) - getPosition();
        mFixedNormal = Vec3d(mpPlaneRep->GetNormal());
        shiftedPos += mFixedPositionShift;
      }
      mPosition = p;
      LOG << "set position\n " << shiftedPos << "\n";
      Vec3d min = shiftedPos - mExtent;
      Vec3d max = shiftedPos + mExtent;
      {
        // compute widget position
        double bounds[6] = { min.x(), max.x(), min.y(), max.y(), min.z(), max.z() };
        mpPlaneRep->PlaceWidget(bounds);
      }
      // compute cropping
      {
        auto im = mpCallback->getImage();
        auto origin = Vec3d(im->GetOrigin());
        auto spacing = Vec3d(im->GetSpacing());

        auto voxelMin = toVoxelCoordinates(origin, spacing, min);
        auto voxelMax = toVoxelCoordinates(origin, spacing, max);
        LOG << " min " << voxelMin << "\n";
        LOG_LINE << " min " << voxelMax;
        mpCallback->setVOI(voxelMin, voxelMax);
        mpCallback->modified();
      }
      mpPlaneRep->SetOrigin(shiftedPos.x(), shiftedPos.y(), shiftedPos.z());
      if (mFixedInteraction)
        mpPlaneRep->SetNormal(mFixedNormal.data());
      mpPlaneRep->Modified();
      setNormal(mNormal);
    }

    /**
    */
    void VisImageOverlay::setNormal(const Vec3d& n)
    {
      Vec3d changedNorm = n;
      if (mFixedInteraction)
      {
        mFixedNormal = Vec3d(mpPlaneRep->GetNormal());
        changedNorm = mFixedNormal;
      }
      mNormal = n;
      if (mOrthSlices)
      {
        auto temp = n;
        temp.normalize();
        temp = temp.abs();
        auto coronal_diff = (Vec3d(1, 0, 0) - temp).squaredNorm();
        auto sagittal_diff = (Vec3d(0, 1, 0) - temp).squaredNorm();
        auto transversal_diff = (Vec3d(0, 0, 1) - temp).squaredNorm();
        if (coronal_diff < sagittal_diff && coronal_diff < transversal_diff)
          changedNorm = Vec3d(1, 0, 0);
        else if (sagittal_diff < transversal_diff && sagittal_diff < coronal_diff)
          changedNorm = Vec3d(0, 1, 0);
        else
          changedNorm = Vec3d(0, 0, 1);
      }
      LOG_LINE << "normal " << changedNorm;
      mpPlaneRep->SetNormal(changedNorm.data());
      mpPlaneRep->Modified();
      mpCallback->modified();
    }
  }
}