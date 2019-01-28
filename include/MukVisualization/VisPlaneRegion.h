#pragma once
#include "VisStateRegion.h"

#include <vtkSmartPointer.h>

namespace gris
{
  namespace muk
  {
    class PlaneRegionRepresentation;
    class PlaneRegionWidget;

    /** \brief A Visualization Object based on a implicit plane widget with modified representation
    */
    class MUK_VIS_API VisPlaneRegion : public VisStateRegion
    {
      public:
        VisPlaneRegion(const std::string& name);
        virtual ~VisPlaneRegion();

        // from VisualObject
      public:
        virtual void setRenderer(vtkRenderer* pRenderer);
        virtual void setVisibility(bool);
        virtual bool getVisibility()      const;

        // from VisStateRegion
      public:
        virtual const MukState& getCenter() const;
        virtual std::unique_ptr<IStateRegion> asStateRegion(bool asStart) const;
        virtual void setFromProblemDefinition(const MukProblemDefinition& obj) {}
        virtual bool isMovable() const;

      public:
        void setPlane(const Vec3d& origin, const Vec3d& normal);
        void setDirectionAncor(const Vec3d& p);
        void setBounds(const std::vector<Vec3d>& points);

      public:
        PlaneRegionWidget*         widget();
        PlaneRegionRepresentation* representation();
        void swap(VisPlaneRegion* other);
        
      private:
        vtkSmartPointer<PlaneRegionWidget>  mpWidget;
        PlaneRegionRepresentation*          mpRep;
        double mResolution;
    };
  }
}
