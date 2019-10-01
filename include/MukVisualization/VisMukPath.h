#pragma once

#include "muk_visualization_api.h"
#include "VisualObject.h"

#include "MukCommon/Waypoints.h"
#include "MukCommon/MukPath.h"

namespace gris
{
  namespace muk
  {

    class MUK_VIS_API VisMukPath : public VisualObject
    {
      public:
        enum EnColorMode
        {
          singleColor,
          multiColor,
        };
        
        /** \brief possible topology for the visualization of a #MukPath (#VisMukPath).
        */
        enum EnTopology
        {
          Vertices,
          Lines,
          Tube,
          N_Topologies
        };

        static const char* s_topologyStr[N_Topologies];
        static std::vector<std::string> getTopologies();

      public:
        explicit VisMukPath(const MukPath& pPath);
        virtual ~VisMukPath();

      public:
        virtual void update();

        MukPath asMukPath();

      public:
        void setData(const MukPath& path);
        using VisualObject::setData;

        void       setTopology(EnTopology i);
        EnTopology getTopology()       const { return mTopology; }
        void       setLineWidth(double d);
        double     getLineWidth() const;

      public:
        void  reloadTopology();

      private:
        EnTopology mTopology;
        const MukPath& mPath;
        double mLineWidth;
    };

  }
}

namespace std
{
  MUK_VIS_API std::ostream& operator<< (std::ostream& os, const gris::muk::VisMukPath::EnTopology& obj); 
  MUK_VIS_API std::istream& operator>> (std::istream& is, gris::muk::VisMukPath::EnTopology& obj);
}
