#pragma once

#include "muk_visualization_api.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukVector.h"

#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkProp3D.h>

class vtkRenderer;

#include <memory>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API VisualProp
    {
      public:
        VisualProp(vtkSmartPointer<vtkProp3D> prop);
        ~VisualProp();

      public:
        virtual void update() = 0;

      public:
        virtual void        setRenderer(vtkRenderer* pRenderer);
        vtkRenderer*        getRenderer()                           const   { return mpRenderer; }
        virtual void        setColors(const std::vector<Vec3d>& colors);

      public:
        virtual void setTransform(const vtkSmartPointer<vtkTransform> transform);
        vtkSmartPointer<vtkTransform> getTransform();
        
      protected:
        vtkRenderer*                mpRenderer;
        vtkSmartPointer<vtkProp3D>  mpActor;
    };

  }
}
