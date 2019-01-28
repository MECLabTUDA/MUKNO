#pragma once
#include "VisStateRegion.h"

namespace gris
{
  namespace muk
  {
    class SurfaceRegionRepresentation;
    class SurfaceRegionWidget;

    /** \brief Visualization of a mesh with normals
    */
    class MUK_VIS_API VisSurfaceRegion : public VisStateRegion
    {
      public:
        VisSurfaceRegion(const std::string& name);
        virtual ~VisSurfaceRegion();
        
      public:
        virtual void setRenderer(vtkRenderer* pRenderer);
        virtual void setVisibility(bool);
        virtual bool getVisibility()      const;
        virtual void        setColors(const Vec3d& color);

      public:
        virtual const MukState& getCenter() const;
        virtual std::unique_ptr<IStateRegion> asStateRegion(bool asStart) const;
        virtual bool isMovable() const;
        virtual void setFromProblemDefinition(const MukProblemDefinition& obj) {}

      public:
        void setSurfacePoints (const std::vector<Vec3d>& p);
        void setDirectionAncor(const Vec3d& p);

      public:
        SurfaceRegionWidget*         widget();
        SurfaceRegionRepresentation* representation();
        void swap(VisSurfaceRegion* other);
        
      private:
        vtkSmartPointer<SurfaceRegionWidget>         mpWidget;
        SurfaceRegionRepresentation* mpRep;
    };
  }
}
