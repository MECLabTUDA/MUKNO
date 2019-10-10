#pragma once
#include "private/muk.pch"
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

class vtkSphereSource;
class vtkConeSource;
class vtkLineSource;
class vtkPolyDataMapper;
class vtkActor;

namespace gris
{
  namespace muk
  {
    /** \brief Custom Interaction with a larger ball and directions instead of small ball of vtkImplicitPlaneRepresentation
      
      overrides the vtkImplicitPlaneRepresentation functions below.
    */
    class ImplicitPlaneRepresentation : public vtkImplicitPlaneRepresentation
    {
      public:
        static ImplicitPlaneRepresentation* New();
        vtkTypeMacro(ImplicitPlaneRepresentation, vtkImplicitPlaneRepresentation);
        vtkGetObjectMacro(mpConeProperty, vtkProperty);
        vtkGetObjectMacro(mpSelectedConeProperty, vtkProperty);

      public:
        ImplicitPlaneRepresentation();
        ~ImplicitPlaneRepresentation();
        
      protected:
        // \brief sets the radius of the origin
        void setSphereRadius(double radius) { mRadius = radius; };

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

      double mRotateMatrix[3][3];  // Rotationmatrix to move Arrowrs similar to the normal
      double mRadius = 20;		// Radius of the Sphere
      double mOldRadius = 20;     // old Radius for changing purposes

      // Cones for Arrow
      vtkConeSource     *mpArrowMotionPlanningConeSource1;
      vtkPolyDataMapper	*mpArrowMotionPlanningConeMapper1;
      vtkActor          *mpArrowMotionPlanningConeActor1;
      vtkConeSource     *mpArrowMotionPlanningConeSource2;
      vtkPolyDataMapper	*mpArrowMotionPlanningConeMapper2;
      vtkActor          *mpArrowMotionPlanningConeActor2;
      vtkConeSource     *mpArrowMotionPlanningConeSource3;
      vtkPolyDataMapper	*mpArrowMotionPlanningConeMapper3;
      vtkActor          *mpArrowMotionPlanningConeActor3;

      // Lines for Arrow
      vtkLineSource     *mpArrowMotionPlanningLineSource1;
      vtkPolyDataMapper	*mpArrowMotionPlanningLineMapper1;
      vtkActor          *mpArrowMotionPlanningLineActor1;
      vtkLineSource     *mpArrowMotionPlanningLineSource2;
      vtkPolyDataMapper	*mpArrowMotionPlanningLineMapper2;
      vtkActor          *mpArrowMotionPlanningLineActor2;
      vtkLineSource     *mpArrowMotionPlanningLineSource3;
      vtkPolyDataMapper	*mpArrowMotionPlanningLineMapper3;
      vtkActor          *mpArrowMotionPlanningLineActor3;

      vtkProperty *mpSelectedConeProperty;
      vtkProperty *mpConeProperty;
    };
  }
}