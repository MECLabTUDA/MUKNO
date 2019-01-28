#pragma once
#include "muk_visualization_api.h"

#include "MukCommon/MukVector.h"

#include <vtkSmartPointer.h>
class vtkActor;
class vtkInteractor;
class vtkInteractorStyle;
class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkOrientationMarkerWidget;
class vtkAxesActor;
class vtkProp3D;

#include <tuple>

namespace gris
{
  namespace muk
  {
    /** \brief Convenience class that builds a basic vtk-visualization window
    */
    class MUK_VIS_API BasicVisualization
    {
      public:
        BasicVisualization();
        ~BasicVisualization();

       public:
        vtkRenderer*      getRenderer();
        vtkRenderWindow*  getWindow();

      public:
        void start();

        size_t addObject(vtkPolyData* pData);
        size_t addProp3D(vtkProp3D* pData);
        void clearObject(size_t idx);
        void clearObjects();
        size_t numObjects() const { return mObjects.size(); }

        vtkActor*           getActor(size_t idx);
        vtkPolyDataMapper*  getMapper(size_t idx);
        
        vtkActor*           objectActor(size_t id);
        vtkPolyDataMapper*  objectMapper(size_t id);
        vtkPolyData*        objectData(size_t id);

        void setColor(size_t id, const Vec3d& color);
        void setOpacity(size_t id, double val);

      public:
        using SimpleObjectSet = std::tuple<vtkSmartPointer<vtkActor>, vtkSmartPointer<vtkPolyDataMapper>, vtkSmartPointer<vtkPolyData>>;

      private:
        vtkSmartPointer<vtkRenderer>        mpRenderer;
        vtkSmartPointer<vtkRenderWindow>    mpWindow;
        vtkSmartPointer<vtkOrientationMarkerWidget> mpAxesOrientation;

        std::vector<SimpleObjectSet> mObjects;
        std::vector<vtkSmartPointer<vtkProp3D>> mPropObjects;
    };
  }
}
