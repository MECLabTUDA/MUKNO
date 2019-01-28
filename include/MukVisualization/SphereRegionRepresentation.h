#pragma once
#include "muk_visualization_api.h"
#include "ArrowTrajectory.h"

#include "MukCommon/MukVector.h"
#include "MukCommon/MukState.h"

#include <vtkArrowSource.h>
#include <vtkBox.h>
#include <vtkCellPicker.h>
#include <vtkSphereSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkWidgetRepresentation.h>

namespace gris
{
  namespace muk
  {
    class ArrowTrajectory;

    /** \brief Representation of a partial sphere and an arrow

      implementation inspired by vtkImplicitPlaneRepresentation
    */
    class MUK_VIS_API SphereRegionRepresentation : public vtkWidgetRepresentation
    {
      public:
        enum EnInteractionState
        {
          enOutside,
          enMovingMouse,
          enMovingSphere,
          enPlaceSphere,
          enRotating,
        };

      public:
        static SphereRegionRepresentation* New();
        vtkTypeMacro(SphereRegionRepresentation, vtkWidgetRepresentation);

      public:
        SphereRegionRepresentation();
        ~SphereRegionRepresentation();

      public:
        void   setHandles(bool on);
        void   setArrowDirection(const Vec3d& v);
        void   setSphereOrigin(const Vec3d& p);
        void   setSpherePhi(double d);
        double getSpherePhi()            const;
        void   setSphereRadius(double d);
        double getSphereRadius()            const;
        MukState getState()                 const;

      public:
        virtual void PlaceWidget(const MukState& state);
        void    SetInteractionState(int en);

        /**
        * Methods supporting the rendering process.
        */
      public:
        virtual int  ComputeInteractionState(int X, int Y, int modify=0);
        virtual void PlaceWidget(double* b);
        virtual void BuildRepresentation();
        virtual void StartWidgetInteraction(double eventPos[2]);
        virtual void WidgetInteraction(double newEventPos[2]);
        virtual void EndWidgetInteraction(double newEventPos[2]);
        virtual void Highlight(int highlight);
        
        /**
        * Methods supporting the rendering process.
        */
      public:
        virtual double *GetBounds();
        //virtual void GetActors(vtkPropCollection *pc);
        virtual void ReleaseGraphicsResources(vtkWindow*);
        virtual int  RenderOpaqueGeometry(vtkViewport*);
        virtual int  RenderTranslucentPolygonalGeometry(vtkViewport*);
        virtual int  HasTranslucentPolygonalGeometry();

      public:
        // Methods to manipulate the plane
        void RotateCone(double X, double Y, double * p1, double * p2, double * coneDirection);
        void TranslateSphere(double *p1, double *p2);
        void placeSphere(double *p1);
        void SizeHandles();
        void   SetScale(double);
        double GetScale() const;

      private:
        struct SphereContainer
        {
          SphereContainer();

          Vec3d origin;
          vtkSmartPointer<vtkSphereSource> sphere;
          vtkSmartPointer<vtkTransform> matrix;
          vtkSmartPointer<vtkTransformPolyDataFilter> trafo;
          vtkSmartPointer<vtkPolyDataMapper> mapper;
          vtkSmartPointer<vtkActor> actor;
          vtkSmartPointer<vtkProperty> propActive;
          vtkSmartPointer<vtkProperty> propInactive;
        };

        struct ArrowContainer
        {
          ArrowContainer();
          Vec3d direction;
          vtkSmartPointer<vtkArrowSource> arrow;
          vtkSmartPointer<vtkTransform> matrix;
          vtkSmartPointer<vtkTransformPolyDataFilter> trafo;
          vtkSmartPointer<vtkPolyDataMapper> mapper;
          vtkSmartPointer<vtkActor> actor;
          vtkSmartPointer<vtkProperty> propActive;
          vtkSmartPointer<vtkProperty> propInactive;
        };

      private:
        void CreateDefaultProperties();
        void HighlightArrow(bool on);

      private:
        double mLastEventPosition[3]; /// Keep track of event positions
        double mBumpDistance;         /// Controlling the push operation
        double mScale = 1.0;

        vtkSmartPointer<vtkBox>        mBoundingBox;
        vtkSmartPointer<vtkCellPicker> mPicker;
        SphereContainer mSphere;
        ArrowContainer  mArrow;

        vtkSmartPointer<vtkTransform> mTransform; /// used for rotation
    };
  }
}