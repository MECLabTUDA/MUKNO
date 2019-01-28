#pragma once
#include "muk_visualization_api.h"
#include "ArrowTrajectory.h"

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
    class ArrowTrajectory;

    /** \brief Custom Interaction with an additional small ball do define the direction

      overrides the vtkPlaneRegionRepresentation functions below.
    */
    class MUK_VIS_API PlaneRegionRepresentation : public vtkImplicitPlaneRepresentation
    {
      public:
        /** Manage the state of the widget
        use vtk conventions to override or maintain functionality of base class
        */
        enum EnInteractionState
        {
          enOutside = 0,
          enMoving,
          enMovingOutline,
          enMovingOrigin,
          enRotating,
          enPushing,
          enScaling,
          enPlaceAncor,
          enPlacePlane,
          enMovingDirectionBall,
        };

      public:
        static PlaneRegionRepresentation* New();
        vtkTypeMacro(PlaneRegionRepresentation, vtkImplicitPlaneRepresentation);

      public:
        PlaneRegionRepresentation();
        ~PlaneRegionRepresentation();

      public:
        void    centralize(const Vec3d& p);
        void    setHandles(bool on);
        Vec3d   getDirectionAncor() const;
        std::vector<Vec3d> getBoundingPolygon() const;
        void    setPlane(const Vec3d& p, const Vec3d& n);
        void    setDirectionAncor(const Vec3d& p);
        void    setBounds(const std::vector<Vec3d>& p);
      
      public:
        // Methods supporting the rendering process.
        virtual int  ComputeInteractionState(int X, int Y, int modify=0); /// \brief checks which Interactionstate is activated
        virtual void BuildRepresentation();
        //virtual void StartWidgetInteraction(double eventPos[2]);
        virtual void WidgetInteraction(double newEventPos[2]);
        //virtual void EndWidgetInteraction(double newEventPos[2]);
        //virtual void Highlight(int highlight);

      public:
        // Methods supporting the rendering process.
        virtual double *GetBounds();
        virtual void GetActors(vtkPropCollection *pc);
        virtual void ReleaseGraphicsResources(vtkWindow*);
        virtual int  RenderOpaqueGeometry(vtkViewport*);
        virtual int  RenderTranslucentPolygonalGeometry(vtkViewport*);
        virtual int  HasTranslucentPolygonalGeometry();

      public:
        // Methods to manipulate the representation
        void TranslateOutline(double *p1, double *p2);
        void TranslateDirectionBall(double *p1, double *p2);
        using vtkImplicitPlaneRepresentation::SetNormal;
        void SetNormal(double n[3]) { this->SetNormal(n[0], n[1], n[2]); }  /// \brief Converts an Array to single values to set the normal
        void PlacePlane(double* e);
        void PlaceSphere(double* e);

        // Methods to manipulate the representation's view
        void SizeHandles();
        void HighlightNormal(int highlight);  /// \brief Hightlights the selected Objects
        void HighlightDirectionBall(int highlight);
        void SetRepresentationState(int state); /// \brief Sets the Representationstate and highlights the coresponding objects

      public:        
        void SetInteractionState(int state);
        // \brief LockNormalToCamera will cause the normal to follow the camera's normal
        void SetLockNormalToCamera(int lock);
        // Used to print the state
        void PrintSelf(ostream& os, vtkIndent indent);
        // \brief Creates the default properties for the Objects
        void CreateDefaultProperties();
        

      private:
        // \brief A Sphere and a line to visualize the ancor points that is used to determine the individual directions
        struct DirectionBall
        {
          DirectionBall();
          vtkSmartPointer<vtkSphereSource>   sphere;
          vtkSmartPointer<vtkPolyDataMapper>     sphereMapper;
          vtkSmartPointer<vtkActor> sphereActor;

          vtkSmartPointer<vtkPolyData> lines;
          vtkSmartPointer<vtkPolyDataMapper> linesMapper;
          vtkSmartPointer<vtkActor> linesActor;

          vtkSmartPointer<vtkProperty> prop;
          vtkSmartPointer<vtkProperty> propSelected;
        };

      private:
        DirectionBall mDirBall;
        ArrowTrajectory mArrows;
    };
  }
}