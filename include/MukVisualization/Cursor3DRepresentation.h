#pragma once
#include "muk_visualization_api.h"

#include "vtkInteractionWidgetsModule.h" // For export macro
#include "vtkWidgetRepresentation.h"
#include "vtkSmartPointer.h"

class vtkActor;
class vtkLineSource;
class vtkSphereSource;
class vtkCellPicker;
class vtkProperty;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkPoints;
class vtkPolyDataAlgorithm;
class vtkPointHandleRepresentation3D;
class vtkTransform;
class vtkPlanes;
class vtkBox;
class vtkDoubleArray;
class vtkMatrix4x4;
class vtkImageData;

#include <array>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API Cursor3DRepresentation : public vtkWidgetRepresentation
    {
    public:
      static Cursor3DRepresentation *New();

      vtkTypeMacro(Cursor3DRepresentation,vtkWidgetRepresentation);

    public:
      Cursor3DRepresentation(const Cursor3DRepresentation&) = delete;
      void operator=(const Cursor3DRepresentation&) = delete;

    protected:
      Cursor3DRepresentation();
      ~Cursor3DRepresentation();

    public:
      virtual void HandlesOn();
      virtual void HandlesOff();

    public:
      virtual void PlaceWidget(double bounds[6]);
      virtual void BuildRepresentation();
      virtual int  ComputeInteractionState(int X, int Y, int modify=0);
      virtual void StartWidgetInteraction(double e[2]);
      virtual void WidgetInteraction(double e[2]);
      virtual double *GetBounds();

    public:
      virtual void ReleaseGraphicsResources(vtkWindow*);
      virtual int  RenderOpaqueGeometry(vtkViewport*);
      virtual int  RenderTranslucentPolygonalGeometry(vtkViewport*);
      virtual int  HasTranslucentPolygonalGeometry();

    public:
      //@}

      // Used to manage the state of the widget
      enum EnStates
      {
        enOutside=0,
        enMoving,
      };

      void SetInteractionState(int state);
      void getPosition(double d[3]);
      void setPosition(double d[3]);
      void setImage(vtkImageData* p) { mpImage = p; }

    private:
      // Helper methods
      virtual void Translate(double *p1, double *p2);

    private:
      struct Line
      {
        Line();

        vtkSmartPointer<vtkPolyData>       lineData;
        vtkSmartPointer<vtkPolyDataMapper> lineMapper;
        vtkSmartPointer<vtkActor>          lineActor;
      };

    private:
      // Manage how the representation appears
      double mLastEventPosition[3];
      double mPosition[3];
      // 6 faces
      std::array<Line, 3> mLines;
      // Support GetBounds() method
      vtkSmartPointer<vtkBox> mBBox;
      vtkImageData* mpImage;
    };
  }
}