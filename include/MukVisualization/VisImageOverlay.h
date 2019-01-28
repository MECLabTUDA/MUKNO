#pragma once

#include "VisAbstractObject.h"

#include <vtkSmartPointer.h>
class vtkImageData;
class vtkImplicitPlaneWidget2;
class vtkImplicitPlaneRepresentation;

namespace gris
{
  namespace muk
  {
    class ImageOverlayCallback;

    /** \brief Aa Visualization Object that renders part of an CT-Slice within a bounding volume
    */
    class MUK_VIS_API VisImageOverlay : public VisAbstractObject
    {
      public:
        VisImageOverlay(const std::string& name);
        virtual ~VisImageOverlay();

      public:
        virtual void setRenderer(vtkRenderer* pRenderer);
        virtual void setOpacity(double d);
        virtual void setVisibility(bool visible);

      public:
        void setImage(vtkImageData* pImage);
        void setPosition(const Vec3d& p);
        const Vec3d& getPosition() { return mPosition; };
        void setNormal(const Vec3d& n);
        const Vec3d& getNormal() { return mNormal; };
        void setFixedInteraction(const bool fixed);

      private:
        void adjustToSize();

      private:
        vtkSmartPointer<vtkImplicitPlaneWidget2> mpImpPlaneWidget;
        vtkSmartPointer<vtkImplicitPlaneRepresentation> mpPlaneRep;
        vtkSmartPointer<ImageOverlayCallback> mpCallback;

        Vec3d  mPosition;
        Vec3d  mExtent;
        Vec3d  mNormal;
        bool   mFixedInteraction = false;
        Vec3d  mFixedPositionShift;
        Vec3d  mFixedNormal;
        bool   mOrthSlices = false;
    };
  }
}
