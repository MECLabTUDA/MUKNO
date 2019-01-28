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

#include <array>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API ROIRepresentation : public vtkWidgetRepresentation
    {
      public:
        static ROIRepresentation *New();

        vtkTypeMacro(ROIRepresentation,vtkWidgetRepresentation);
        void PrintSelf(ostream& os, vtkIndent indent);

      public:
        ROIRepresentation(const ROIRepresentation&) = delete;
        void operator=(const ROIRepresentation&) = delete;

      protected:
        ROIRepresentation();
        ~ROIRepresentation();

      public:
        void GetPlanes(vtkPlanes *planes);
        vtkSetMacro(InsideOut,int);
        vtkGetMacro(InsideOut,int);
        vtkBooleanMacro(InsideOut,int);
        virtual void GetTransform(vtkTransform *t);
        virtual void SetTransform(vtkTransform* t);
        void GetPolyData(vtkPolyData *pd);
        void SetOutlineFaceWires(int);
        vtkGetMacro(OutlineFaceWires,int);
        void OutlineFaceWiresOn() {this->SetOutlineFaceWires(1);}
        void OutlineFaceWiresOff() {this->SetOutlineFaceWires(0);}
        void SetOutlineCursorWires(int);
        vtkGetMacro(OutlineCursorWires,int);
        void OutlineCursorWiresOn() {this->SetOutlineCursorWires(1);}
        void OutlineCursorWiresOff() {this->SetOutlineCursorWires(0);}

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
        //@}

        // Used to manage the state of the widget
        enum EnStates
        {
          Outside=0,
          MoveF0,
          MoveF1,
          MoveF2,
          MoveF3,
          MoveF4,
          MoveF5,
          Translating,
          Rotating,
          Scaling
        };

        void SetInteractionState(int state);

      private:
        virtual void  PositionHandles();
        virtual void  ComputeNormals();
        virtual void  SizeHandles();
        virtual void  CreateDefaultProperties();

      public:
        void updateVisibility(double position[3]);

      private:
        int           HighlightHandle(vtkProp *prop); //returns cell id
        void          HighlightFace(int cellId);
        void          HighlightOutline(int highlight);

        void GenerateOutline();

        // Helper methods
        virtual void Translate(double *p1, double *p2);
        virtual void Scale(double *p1, double *p2, int X, int Y);
        virtual void Rotate(int X, int Y, double *p1, double *p2, double *vpn);
        void MovePlusXFace(double *p1, double *p2);
        void MoveMinusXFace(double *p1, double *p2);
        void MovePlusYFace(double *p1, double *p2);
        void MoveMinusYFace(double *p1, double *p2);
        void MovePlusZFace(double *p1, double *p2);
        void MoveMinusZFace(double *p1, double *p2);

        //"dir" is the direction in which the face can be moved i.e. the axis passing
        //through the center
        void MoveFace(double *p1, double *p2, double *dir,
          double *x1, double *x2, double *x3, double *x4,
          double *x5);
        //Helper method to obtain the direction in which the face is to be moved.
        //Handles special cases where some of the scale factors are 0.
        void GetDirection(const double Nx[3],const double Ny[3],
          const double Nz[3], double dir[3]);

      private:
        struct Face
        {
          Face();
          void setFace(vtkIdType i1, vtkIdType i2, vtkIdType i3, vtkIdType i4);

          vtkSmartPointer<vtkPolyData>       faceData;
          vtkSmartPointer<vtkPolyDataMapper> faceMapper;
          vtkSmartPointer<vtkActor>          faceActor;
          
          vtkSmartPointer<vtkPolyData>       outlineData;
          vtkSmartPointer<vtkPolyDataMapper> outlineMapper;
          vtkSmartPointer<vtkActor>          outlineActor;
        };

      private:
        // Manage how the representation appears
        double LastEventPosition[3];
        
        // 6 faces
        std::array<Face,6> mFaces;

        vtkPoints*         Points;  //used by others as well
        double             mNormals[6][3]; //the normals of the faces
        
        // Do the picking
        vtkCellPicker *HandlePicker;
        vtkCellPicker *HexPicker;
        vtkActor *CurrentHandle;
        int      CurrentHexFace;
        vtkCellPicker *LastPicker;

        // Register internal Pickers within PickingManager
        virtual void RegisterPickers();

        // Transform the hexahedral points (used for rotations)
        vtkTransform *Transform;

        // Support GetBounds() method
        vtkBox *BoundingBox;

        // Properties used to control the appearance of selected objects and
        // the manipulator in general.
        vtkSmartPointer<vtkProperty> mHandleProperty;
        vtkSmartPointer<vtkProperty> mSelectedHandleProperty;

        vtkSmartPointer<vtkProperty> mFaceProperty;
        vtkSmartPointer<vtkProperty> mSelectedFaceProperty;
        vtkSmartPointer<vtkProperty> mOutlineProperty;
        vtkSmartPointer<vtkProperty> mSelectedOutlineProperty;

        // Control the orientation of the normals
        int InsideOut;
        int OutlineFaceWires;
        int OutlineCursorWires;

        // Internal ivars for performance
        vtkPoints      *PlanePoints;
        vtkDoubleArray *PlaneNormals;
        vtkMatrix4x4   *Matrix;
    };
  }
}