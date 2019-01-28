#pragma once

#include "muk_visualization_api.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukVector.h"

#include "gstd/dynamicProperty.h"

#include <vtkSmartPointer.h>
class vtkRenderer;
class vtkActor;
class vtkPolyDataMapper;
class vtkPolyData;

#include <memory>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API VisualObject : public gstd::DynamicProperty
    {
      public:
        VisualObject();
        explicit VisualObject(const gstd::DynamicProperty& prop);
        virtual ~VisualObject();

      public:
        virtual void update() = 0;

      public:
        virtual void        setRenderer(vtkRenderer* pRenderer);
        vtkRenderer*        getRenderer()                           const   { return mpRenderer; }
        virtual void        setColors(const std::vector<Vec3d>& colors);
        virtual void        setColors(double color[3]);
        virtual void        setColors(const Vec3d& color);

        void                setDefaultColor(const Vec3d& color);
        void                setDefaultColor(double color[3]);

        const Vec3d&        getDefaultColor()                       const;
        virtual void        setVisibility(bool);
        virtual bool        getVisibility()                         const;
        virtual void        setOpacity(double);
        double              getOpacity()                            const;
        virtual void        setData(vtkSmartPointer<vtkPolyData> data);
        vtkSmartPointer<vtkPolyData> getData();
        
      protected:
        void declareProperties();

      protected:
        const gstd::DynamicProperty* mProp;

        Vec3d                       mDefaultColor;

        vtkRenderer*                mpRenderer;
        vtkSmartPointer<vtkActor>           mpActor;
        vtkSmartPointer<vtkPolyDataMapper>  mpMapper;
        vtkSmartPointer<vtkPolyData>        mpData;

        std::vector<vtkSmartPointer<vtkActor>>          mpActors;
        std::vector<vtkSmartPointer<vtkPolyDataMapper>> mpMappers;
        std::vector<vtkSmartPointer<vtkPolyData>>       mpDatas;
    };

  }
}
