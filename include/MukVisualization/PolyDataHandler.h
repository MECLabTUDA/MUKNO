#pragma once

#include "muk_visualization_api.h"

#include "MukCommon/MukVector.h"

class vtkPolyData;

namespace gris
{
  namespace muk
  {

    struct MUK_VIS_API PolyDataHandler
    {    
      public:
        explicit PolyDataHandler(vtkPolyData* p) : mpData(p) {}
        
      public:
        static void clearTopology (vtkPolyData* data);
        static void addVertices   (vtkPolyData* data);
        static void addLines      (vtkPolyData* data, double lineWidth = 1.0);

      public:
        void clearTopology ();
        void addVertices   ();
        void addLines      ();

        size_t size() const;

        Vec3d getPoint (size_t idx)           const;
        void  getPoint (Vec3d& v, size_t idx) const;  // convenience function

        VecN3d  getPoints()          const;
        void    getPoints(VecN3d& v) const;
        
        
        

      private:
        vtkPolyData* mpData;      
    };

  }
}
