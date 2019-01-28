#pragma once
#include "muk_visualization_api.h"

#include "MukCommon/MukVector.h"
#include "MukCommon/MukState.h"

#include <vtkImplicitPlaneRepresentation.h>
#include <vtkSphereSource.h>
#include <vtkConeSource.h>
#include <vtkTubeFilter.h>
#include <vtkFeatureEdges.h>
#include <vtkOutlineFilter.h>
#include <vtkPlane.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkImageData.h>
#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkCellPicker.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

class vtkSphereSource;
class vtkConeSource;
class vtkLineSource;
class vtkPolyDataMapper;
class vtkTransformPolyDataFilter;
class vtkActor;

namespace gris
{
  namespace muk
  {
    /** \brief Custom Interaction with a larger ball and directions instead of small ball of vtkMultiPortSphereRegionRepresentation

    overrides the vtkMultiPortSphereRegionRepresentation functions below.
    */
    class MUK_VIS_API MultiPortSphereRegionRepresentation : public vtkImplicitPlaneRepresentation
    {
      public:
        static MultiPortSphereRegionRepresentation* New();
        vtkTypeMacro(MultiPortSphereRegionRepresentation, vtkImplicitPlaneRepresentation);
        vtkGetObjectMacro(mpArrowProperty, vtkProperty);
        vtkGetObjectMacro(mpSelectedArrowProperty, vtkProperty);

      public:
        MultiPortSphereRegionRepresentation();
        ~MultiPortSphereRegionRepresentation();

      public:
        void    centralize(const Vec3d& p);
        void    setSphereRadius(double radius);
        void    setPathRadius(double radius);
        void    setAngleThreshold(double alpha);
        void    setPortActivity(const Vec3i& p);
        void    setHandleVisibility(bool active);

        double  getSphereRadius()       const;
        double  getPathRadius()         const { return mPathRadius; }
        double  getAngleThreshold()     const { return mAngleThreshold; }
        const Vec3i& getPortActivity()  const { return mPortActivity; }
        MukState     getState(size_t i);

        /** Manage the state of the widget
          use vtk conventions to override or maintain functionality of base class
        */
        enum _InteractionState
        {
          Outside = 0,
          Moving,
          MovingOutline,
          MovingOrigin,
          Rotating,
          Pushing,
          Scaling,
          RotatingArrow1,
          RotatingArrow2,
          RotatingArrow3
        };

      public:
        // \brief Builds the the representation based on the normal and origin of the plane
        void BuildRepresentation();

        // \brief Manages the the size of the Cones if Zoomed
        void SizeHandles();

        // \brief rotates the Plane to the Arrowmotion
        void Rotate(double X, double Y, double *p1, double *p2, double *vpn);

        // \brief rotates an Arrow
        void RotateCone(double X, double Y, double *p1, double *p2, double *coneDirection);

        // Scales Origin and Box
        void Scale(double *p1, double *p2, double vtkNotUsed(X), double Y, double * vpn);

        // \brief Set the normal to the plane and calculates the rotationmatrix
        void SetNormal(double x, double y, double z);

        // \brief Converts an Array to single values to set the normal
        void SetNormal(double n[3]) { this->SetNormal(n[0], n[1], n[2]); }

        // \brief LockNormalToCamera will cause the normal to follow the camera's normal
        void SetLockNormalToCamera(int lock);

        // \brief checks wich Interactionstate is activated
        int ComputeInteractionState(int X, int Y, int vtkNotUsed(modify));

        // \brief Sets the Representationstate and highlights the coresponding objects
        void SetRepresentationState(int state);

        // \brief Starts calculation for the Interactions
        void WidgetInteraction(double e[2]);

        // \brief returns the bounds
        double *GetBounds();

        // \brief returns the actors
        void GetActors(vtkPropCollection *pc);

        // \brief Release graphics resources and ask components to release their own resources
        void ReleaseGraphicsResources(vtkWindow *w);

        // Support the standard render methods. 
        int RenderOpaqueGeometry(vtkViewport *v);

        // Support the standard render methods. 
        int RenderTranslucentPolygonalGeometry(vtkViewport *v);

        // Support the standard render methods. 
        int HasTranslucentPolygonalGeometry();

        // Used to print the state
        void PrintSelf(ostream& os, vtkIndent indent);
        
        // \brief Creates the default properties for the Objects
        void CreateDefaultProperties();

        // \brief Hightlights the selected Objects
        void HighlightNormal(int highlight);

      private:
        struct Arrow
        {
          Arrow();

          // Cones for Arrow
          // Lines for Arrow
          vtkSmartPointer<vtkLineSource>     mLine;
          vtkSmartPointer<vtkTubeFilter>     mTube;
          vtkSmartPointer<vtkPolyDataMapper> mTubeMapper;
          vtkSmartPointer<vtkActor>          mTubeActor;

          vtkSmartPointer<vtkPolyDataMapper> mLineMapper;
          vtkSmartPointer<vtkActor>          mLineActor;

          vtkSmartPointer<vtkSphereSource>   mThresh;
          vtkSmartPointer<vtkTransformPolyDataFilter> mThreshTrafo;
          vtkSmartPointer<vtkPolyDataMapper> mThreshMapper;
          vtkSmartPointer<vtkActor>          mThreshActor;

          Vec3d mDirection;

          vtkLineSource*      line()       { return mLine.Get(); }
          vtkTubeFilter*      tube()       { return mTube.Get(); }
          vtkPolyDataMapper*  tubeMapper() { return mTubeMapper.Get(); }
          vtkActor*           tubeActor()  { return mTubeActor.Get(); }

          vtkPolyDataMapper*  lineMapper() { return mLineMapper.Get(); }
          vtkActor*           lineActor()  { return mLineActor.Get(); }
        };

      private:
        double mPathRadius = 1;		// Radius of the three access arrows#
        double mAngleThreshold = 0.1;
        Vec3i  mPortActivity;
        bool   mRenderHandles = true;
        
        Arrow mArrows[3];        
        vtkSmartPointer<vtkProperty> mpSelectedArrowProperty;
        vtkSmartPointer<vtkProperty> mpArrowProperty;
    };
  }
}