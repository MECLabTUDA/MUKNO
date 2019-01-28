#pragma once
#include "muk_visualization_api.h"

#include "MukCommon/MukVector.h"
#include "MukCommon/MukState.h"

#include "MukVisualization/ArrowTrajectory.h"

#include <vtkArrowSource.h>
#include <vtkBox.h>
#include <vtkButtonWidget.h>
#include <vtkCellPicker.h>
#include <vtkInteractorStyleDrawPolygon.h>
#include <vtkLineSource.h>
#include <vtkOutlineFilter.h>
#include <vtkSphereSource.h>
#include <vtkTexturedButtonRepresentation2D.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkWidgetRepresentation.h>

namespace gris
{
  namespace muk
  {
    struct SurfaceRegionRepresentationCallback;

    /**
    */
    class MUK_VIS_API SurfaceRegionRepresentation : public vtkWidgetRepresentation
    {
      public:
        enum EnInteractionState
        {
          enOutside = 0,
          enMoveSphere,
          enScrollSphere,
          enPlaceSphere,
        };

      public:
        static SurfaceRegionRepresentation* New();
        vtkTypeMacro(SurfaceRegionRepresentation, vtkWidgetRepresentation);

      public:
        // methods to set the representation from the state region this rep is based on
        void setDirectionAncor(const Vec3d& p);
        void setSurfacePoints(const std::vector<Vec3d>& p);
        Vec3d              getDirectionAncor() const;
        std::vector<Vec3d> getSurfacePoints()  const;
        void               setArrowColor(const Vec3d& color);

      public:
        void setCache(vtkInteractorObserver* pObj);
        void SetInteractionState(int state);
        void PlaceWidget(const Vec3d& p);
        void setHandles(bool on);

      public:
        // These are methods that satisfy vtkWidgetRepresentation's API.
        virtual void SetRenderer(vtkRenderer *ren);
        virtual void BuildRepresentation();

        virtual void PlaceWidget(double* bounds);
        virtual void StartWidgetInteraction(double e[2]);
        virtual void WidgetInteraction(double e[2]);
        virtual int  ComputeInteractionState(int X, int Y, int modify=0);
        //virtual void EndWidgetInteraction(double newEventPos[2]);
        virtual void Highlight(int highlight);

      public:
        // Methods to make this class behave as a vtkProp.
        virtual double *GetBounds();
        //virtual void GetActors(vtkPropCollection *) {}
        virtual void ReleaseGraphicsResources(vtkWindow* w);
        virtual int  RenderOpaqueGeometry(vtkViewport* v);
        virtual int  RenderTranslucentPolygonalGeometry(vtkViewport* v);
        virtual int  HasTranslucentPolygonalGeometry();

      public:
        virtual void SizeHandles();
        virtual void CreateDefaultProperties();

      protected:
        SurfaceRegionRepresentation();
        ~SurfaceRegionRepresentation() {}

        // the different actions
        void moveSphere(double *p1, double *p2);
        void scrollSphere(double dist);
        void placeSphere(double* picked);
        void setSelectedPoints(vtkSmartPointer<vtkPoints> points);
        void placeButtonWidget();
        void placeArrows();

        void HighlightSphere(int);

      private:
        struct SphereContainer
        {
          SphereContainer();

          vtkSmartPointer<vtkSphereSource> sphere;
          vtkSmartPointer<vtkPolyDataMapper> mapper;
          vtkSmartPointer<vtkActor> actor;

          vtkSmartPointer<vtkProperty> propActive;
          vtkSmartPointer<vtkProperty> propInactive;
        };

        struct LineContainer
        {
          LineContainer();

          vtkSmartPointer<vtkLineSource> line;
          vtkSmartPointer<vtkPolyDataMapper> mapper;
          vtkSmartPointer<vtkActor> actor;

          vtkSmartPointer<vtkProperty> propActive;
          vtkSmartPointer<vtkProperty> propInactive;
        };

        struct PolygonContainer
        {
          PolygonContainer();

          vtkSmartPointer<vtkButtonWidget> buttonWidget;
          vtkSmartPointer<vtkTexturedButtonRepresentation2D> buttonRep;
          vtkSmartPointer<SurfaceRegionRepresentationCallback> callback;
          unsigned long observerTag;

          vtkSmartPointer<vtkPolyData> selectedPoints;
          vtkSmartPointer<vtkPolyDataMapper> selectedPointsMapper;
          vtkSmartPointer<vtkActor> selectedPointsActor;
        };

        struct ArrowContainer
        {
          ArrowContainer();

          ArrowTrajectory arrows;

          vtkSmartPointer<vtkProperty> propActive;
          vtkSmartPointer<vtkProperty> propInactive;
        };


      private:
        double mLastEventPosition[3]; /// Manage how the representation appears
        vtkSmartPointer<vtkCellPicker> mHandlePicker;
        vtkSmartPointer<vtkProperty>   mOutlineProperty;
        vtkSmartPointer<vtkBox>        mBoundingBox;
        vtkSmartPointer<vtkCellPicker> mLastPicker;
        vtkSmartPointer<vtkActor>      mCurrentHandle;
        //vtkSmartPointer<vtkPoints>     mPoints;  //used by others as well

        SphereContainer mSphere;
        LineContainer   mLine;
        PolygonContainer mPoly;
        ArrowContainer   mArrows;
        double mScaleRation;
    };
  }
}