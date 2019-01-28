#pragma once

#include "VisualObject.h"

#include "MukCommon/Waypoints.h"
#include "MukCommon/MukPath.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API VisWaypoints : public VisualObject
    {
      public:
        enum EnColorMode
        {
          singleColor,
          multiColor,
        };

        enum EnTopology
        {
          arrows,
          lines,
          N_Topologies
        };

      static const char* s_topologyStr[N_Topologies];
      static std::vector<std::string> getTopologies();

      public:
        explicit VisWaypoints(const Waypoints& wp);
        virtual ~VisWaypoints();

      public:
        virtual void update();

      public:
        void setData(const Waypoints& wp);
        using VisualObject::setData;        

        //Waypoints asWaypoints() const;

        void        setTopology(EnTopology i);
        EnTopology  getTopology()       const { return mTopology; }
        void        setRadius(double r)       { mRadius = r; }
        double      getRadius()         const { return mRadius; }

      public:        
        virtual void setColors(const std::vector<Vec3d>& colors);
        void      insertState (size_t idx, const Vec3d& point);
        void      setPosition (size_t idx, const Vec3d& point);
        void      setDirection(size_t idx, const Vec3d& direction);
        void      deleteState (size_t idx);
        MukState  getState    (size_t idx);
        bool      isChanged()   const { return changed; }

      private:
        void  reloadTopology();

      private:
        double      mRadius;
        EnTopology  mTopology;
        const Waypoints*     mWaypoints;
        vtkSmartPointer<vtkPolyData> mpDirections;
        // for manipulation
        bool changed;
    };
  }
}

namespace std
{
  MUK_VIS_API std::ostream& operator<< (std::ostream& os, const gris::muk::VisWaypoints::EnTopology& obj);
  MUK_VIS_API std::istream& operator>> (std::istream& is, gris::muk::VisWaypoints::EnTopology& obj);
}
