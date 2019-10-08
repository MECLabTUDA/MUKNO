#include "private/muk.pch"
#include "ROIRepresentation.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include "vtkActor.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyData.h"
#include "vtkCallbackCommand.h"
#include "vtkBox.h"
#include "vtkPickingManager.h"
#include "vtkPolyData.h"
#include "vtkProperty.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkInteractorObserver.h"
#include "vtkMath.h"
#include "vtkCellArray.h"
#include "vtkCellPicker.h"
#include "vtkTransform.h"
#include "vtkDoubleArray.h"
#include "vtkBox.h"
#include "vtkPlanes.h"
#include "vtkCamera.h"
#include "vtkAssemblyPath.h"
#include "vtkWindow.h"
#include "vtkObjectFactory.h"

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(ROIRepresentation);

  //----------------------------------------------------------------------------

  ROIRepresentation::Face::Face()
  {
    faceData = make_vtk<vtkPolyData>();
    faceMapper = make_vtk<vtkPolyDataMapper>();
    faceActor = make_vtk<vtkActor>();
    faceMapper->SetInputData(faceData);
    faceActor->SetMapper(faceMapper);

    outlineData = make_vtk<vtkPolyData>();
    outlineMapper = make_vtk<vtkPolyDataMapper>();
    outlineActor = make_vtk<vtkActor>();
    outlineMapper->SetInputData(outlineData);
    outlineActor->SetMapper(outlineMapper);
  }

  void ROIRepresentation::Face::setFace(vtkIdType i1, vtkIdType i2, vtkIdType i3, vtkIdType i4)
  {
    // faces
    auto cell = make_vtk<vtkCellArray>();
    {
      vtkIdType pts[4];
      pts[0] = i1; 
      pts[1] = i2; 
      pts[2] = i3; 
      pts[3] = i4;
      cell->InsertNextCell(4, pts);
    }
    faceData->SetPolys(cell);
    // outline
    auto lines = make_vtk<vtkCellArray>();    
    {
      vtkIdType pts[2];
      pts[0] = i1; pts[1] = i2;
      lines->InsertNextCell(2,pts);
      pts[0] = i2; pts[1] = i3;
      lines->InsertNextCell(2,pts);
      pts[0] = i3; pts[1] = i4;
      lines->InsertNextCell(2,pts);
      pts[0] = i4; pts[1] = i1;
      lines->InsertNextCell(2,pts);
    }
    outlineData->SetLines(lines);
  }

  //----------------------------------------------------------------------------
  ROIRepresentation::ROIRepresentation()
  {
    // The initial state
    InteractionState = ROIRepresentation::Outside;

    // Handle size is in pixels for this widget
    HandleSize = 5.0;

    // Control orientation of normals
    InsideOut = 0;
    OutlineFaceWires = 0;
    OutlineCursorWires = 1;

    // Set up the initial properties
    CreateDefaultProperties();

    //// Construct initial points
    Points = vtkPoints::New(VTK_DOUBLE);
    Points->SetNumberOfPoints(15);//8 corners; 6 faces; 1 center

    // Construct the faces
    for (int i(0); i< mFaces.size(); ++i)
    {
      mFaces[i].faceData->SetPoints(Points);
      mFaces[i].outlineData->SetPoints(Points);
    }
    {
      mFaces[0].setFace(3,0,4,7);
      mFaces[1].setFace(1,2,6,5);
      mFaces[2].setFace(0,1,5,4);
      mFaces[3].setFace(2,3,7,6);
      mFaces[4].setFace(0,3,2,1);
      mFaces[5].setFace(4,5,6,7);
    }

    for (int i(0); i< mFaces.size(); ++i)
    {
      mFaces[i].faceActor->SetProperty(mFaceProperty);
      mFaces[i].outlineActor->SetProperty(mOutlineProperty);
    }

    // Create the outline
    GenerateOutline();
    
    // Define the point coordinates
    double bounds[6];
    bounds[0] = -0.5;
    bounds[1] = 0.5;
    bounds[2] = -0.5;
    bounds[3] = 0.5;
    bounds[4] = -0.5;
    bounds[5] = 0.5;
    // Points 8-14 are down by PositionHandles();
    BoundingBox = vtkBox::New();
    PlaceWidget(bounds);

    //Manage the picking stuff
    HandlePicker = vtkCellPicker::New();
    HandlePicker->SetTolerance(0.01);

    for (int i(0); i<mFaces.size(); ++i)
      HandlePicker->AddPickList(mFaces[i].outlineActor);

    HandlePicker->PickFromListOn();

    HexPicker = vtkCellPicker::New();
    HexPicker->SetTolerance(0.001);
    HexPicker->PickFromListOn();

    CurrentHandle = nullptr;

    // Internal data memebers for performance
    Transform = vtkTransform::New();
    PlanePoints = vtkPoints::New(VTK_DOUBLE);
    PlanePoints->SetNumberOfPoints(6);
    PlaneNormals = vtkDoubleArray::New();
    PlaneNormals->SetNumberOfComponents(3);
    PlaneNormals->SetNumberOfTuples(6);
    Matrix = vtkMatrix4x4::New();
  }

  //----------------------------------------------------------------------------
  ROIRepresentation::~ROIRepresentation()
  {
    HandlePicker->Delete();
    HexPicker->Delete();

    Transform->Delete();
    BoundingBox->Delete();
    PlanePoints->Delete();
    PlaneNormals->Delete();
    Matrix->Delete();
  }

  //----------------------------------------------------------------------
  void ROIRepresentation::GetPolyData(vtkPolyData *pd)
  {
  }

  //----------------------------------------------------------------------
  void ROIRepresentation::StartWidgetInteraction(double e[2])
  {
    // Store the start position
    StartEventPosition[0] = e[0];
    StartEventPosition[1] = e[1];
    StartEventPosition[2] = 0.0;

    // Store the start position
    LastEventPosition[0] = e[0];
    LastEventPosition[1] = e[1];
    LastEventPosition[2] = 0.0;

    ComputeInteractionState(static_cast<int>(e[0]),static_cast<int>(e[1]),0);
  }

  //----------------------------------------------------------------------
  void ROIRepresentation::WidgetInteraction(double e[2])
  {
    // Convert events to appropriate coordinate systems
    vtkCamera *camera = Renderer->GetActiveCamera();
    if ( !camera )
    {
      return;
    }
    double focalPoint[4], pickPoint[4], prevPickPoint[4];
    double z, vpn[3];
    camera->GetViewPlaneNormal(vpn);

    // Compute the two points defining the motion vector
    double pos[3];
    if ( LastPicker == HexPicker )
    {
      HexPicker->GetPickPosition(pos);
    }
    else
    {
      HandlePicker->GetPickPosition(pos);
    }
    vtkInteractorObserver::ComputeWorldToDisplay(Renderer,
      pos[0], pos[1], pos[2],
      focalPoint);
    z = focalPoint[2];
    vtkInteractorObserver::ComputeDisplayToWorld(Renderer,LastEventPosition[0],
      LastEventPosition[1], z, prevPickPoint);
    vtkInteractorObserver::ComputeDisplayToWorld(Renderer, e[0], e[1], z, pickPoint);

    // Process the motion
    if ( InteractionState == ROIRepresentation::MoveF0 )
    {
      MoveMinusXFace(prevPickPoint,pickPoint);
    }

    else if ( InteractionState == ROIRepresentation::MoveF1 )
    {
      MovePlusXFace(prevPickPoint,pickPoint);
    }

    else if ( InteractionState == ROIRepresentation::MoveF2 )
    {
      MoveMinusYFace(prevPickPoint,pickPoint);
    }

    else if ( InteractionState == ROIRepresentation::MoveF3 )
    {
      MovePlusYFace(prevPickPoint,pickPoint);
    }

    else if ( InteractionState == ROIRepresentation::MoveF4 )
    {
      MoveMinusZFace(prevPickPoint,pickPoint);
    }

    else if ( InteractionState == ROIRepresentation::MoveF5 )
    {
      MovePlusZFace(prevPickPoint,pickPoint);
    }

    else if ( InteractionState == ROIRepresentation::Translating )
    {
      Translate(prevPickPoint, pickPoint);
    }

    else if ( InteractionState == ROIRepresentation::Scaling )
    {
      Scale(prevPickPoint, pickPoint, static_cast<int>(e[0]), static_cast<int>(e[1]));
    }

    else if ( InteractionState == ROIRepresentation::Rotating )
    {
      Rotate(static_cast<int>(e[0]), static_cast<int>(e[1]), prevPickPoint, pickPoint, vpn);
    }

    // Store the start position
    LastEventPosition[0] = e[0];
    LastEventPosition[1] = e[1];
    LastEventPosition[2] = 0.0;
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::MoveFace(double *p1, double *p2, double *dir,
    double *x1, double *x2, double *x3, double *x4,
    double *x5)
  {
    int i;
    double v[3], v2[3];

    for (i=0; i<3; i++)
    {
      v[i] = p2[i] - p1[i];
      v2[i] = dir[i];
    }

    vtkMath::Normalize(v2);
    double f = vtkMath::Dot(v,v2);

    for (i=0; i<3; i++)
    {
      v[i] = f*v2[i];

      x1[i] += v[i];
      x2[i] += v[i];
      x3[i] += v[i];
      x4[i] += v[i];
      x5[i] += v[i];
    }
    PositionHandles();
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::GetDirection(const double Nx[3],const double Ny[3],
    const double Nz[3], double dir[3])
  {
    double dotNy, dotNz;
    double y[3];

    if(vtkMath::Dot(Nx,Nx)!=0)
    {
      dir[0] = Nx[0];
      dir[1] = Nx[1];
      dir[2] = Nx[2];
    }
    else
    {
      dotNy = vtkMath::Dot(Ny,Ny);
      dotNz = vtkMath::Dot(Nz,Nz);
      if(dotNy != 0 && dotNz != 0)
      {
        vtkMath::Cross(Ny,Nz,dir);
      }
      else if(dotNy != 0)
      {
        //dir must have been initialized to the
        //corresponding coordinate direction before calling
        //this method
        vtkMath::Cross(Ny,dir,y);
        vtkMath::Cross(y,Ny,dir);
      }
      else if(dotNz != 0)
      {
        //dir must have been initialized to the
        //corresponding coordinate direction before calling
        //this method
        vtkMath::Cross(Nz,dir,y);
        vtkMath::Cross(y,Nz,dir);
      }
    }
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::MovePlusXFace(double *p1, double *p2)
  {
    double *pts = static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);

    double *h1 = pts + 3*9;

    double *x1 = pts + 3*1;
    double *x2 = pts + 3*2;
    double *x3 = pts + 3*5;
    double *x4 = pts + 3*6;

    double dir[3] = { 1 , 0 , 0};
    ComputeNormals();
    GetDirection(mNormals[1],mNormals[3],mNormals[5],dir);
    MoveFace(p1,p2,dir,x1,x2,x3,x4,h1);
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::MoveMinusXFace(double *p1, double *p2)
  {
    double *pts =
      static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);

    double *h1 = pts + 3*8;

    double *x1 = pts + 3*0;
    double *x2 = pts + 3*3;
    double *x3 = pts + 3*4;
    double *x4 = pts + 3*7;

    double dir[3]={-1,0,0};
    ComputeNormals();
    GetDirection(mNormals[0],mNormals[4],mNormals[2],dir);

    MoveFace(p1,p2,dir,x1,x2,x3,x4,h1);
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::MovePlusYFace(double *p1, double *p2)
  {
    double *pts =
      static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);

    double *h1 = pts + 3*11;

    double *x1 = pts + 3*2;
    double *x2 = pts + 3*3;
    double *x3 = pts + 3*6;
    double *x4 = pts + 3*7;

    double dir[3]={0,1,0};
    ComputeNormals();
    GetDirection(mNormals[3],mNormals[5],mNormals[1],dir);

    MoveFace(p1,p2,dir,x1,x2,x3,x4,h1);
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::MoveMinusYFace(double *p1, double *p2)
  {
    double *pts =
      static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);

    double *h1 = pts + 3*10;

    double *x1 = pts + 3*0;
    double *x2 = pts + 3*1;
    double *x3 = pts + 3*4;
    double *x4 = pts + 3*5;

    double dir[3] = {0, -1, 0};
    ComputeNormals();
    GetDirection(mNormals[2],mNormals[0],mNormals[4],dir);

    MoveFace(p1,p2,dir,x1,x2,x3,x4,h1);
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::MovePlusZFace(double *p1, double *p2)
  {
    double *pts =
      static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);

    double *h1 = pts + 3*13;

    double *x1 = pts + 3*4;
    double *x2 = pts + 3*5;
    double *x3 = pts + 3*6;
    double *x4 = pts + 3*7;

    double dir[3]={0,0,1};
    ComputeNormals();
    GetDirection(mNormals[5],mNormals[1],mNormals[3],dir);

    MoveFace(p1,p2,dir,x1,x2,x3,x4,h1);
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::MoveMinusZFace(double *p1, double *p2)
  {
    double *pts =
      static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);

    double *h1 = pts + 3*12;

    double *x1 = pts + 3*0;
    double *x2 = pts + 3*1;
    double *x3 = pts + 3*2;
    double *x4 = pts + 3*3;

    double dir[3]={0,0,-1};
    ComputeNormals();
    GetDirection(mNormals[4],mNormals[2],mNormals[0],dir);

    MoveFace(p1,p2,dir,x1,x2,x3,x4,h1);
  }

  //----------------------------------------------------------------------------
  // Loop through all points and translate them
  void ROIRepresentation::Translate(double *p1, double *p2)
  {
    double *pts = static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);
    double v[3];

    v[0] = p2[0] - p1[0];
    v[1] = p2[1] - p1[1];
    v[2] = p2[2] - p1[2];

    // Move the corners
    for (int i=0; i<8; i++)
    {
      *pts++ += v[0];
      *pts++ += v[1];
      *pts++ += v[2];
    }
    PositionHandles();
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::Scale(double *vtkNotUsed(p1), double *vtkNotUsed(p2), int vtkNotUsed(X), int Y)
  {
    double *pts = static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);
    double *center = static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(3*14);
    double sf;

    if ( Y > LastEventPosition[1] )
    {
      sf = 1.03;
    }
    else
    {
      sf = 0.97;
    }

    // Move the corners
    for (int i=0; i<8; i++, pts+=3)
    {
      pts[0] = sf * (pts[0] - center[0]) + center[0];
      pts[1] = sf * (pts[1] - center[1]) + center[1];
      pts[2] = sf * (pts[2] - center[2]) + center[2];
    }
    PositionHandles();
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::ComputeNormals()
  {
    double *pts = static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);
    double *p0 = pts;
    double *px = pts + 3*1;
    double *py = pts + 3*3;
    double *pz = pts + 3*4;
    int i;

    for (i=0; i<3; i++)
    {
      mNormals[0][i] = p0[i] - px[i];
      mNormals[2][i] = p0[i] - py[i];
      mNormals[4][i] = p0[i] - pz[i];
    }
    vtkMath::Normalize(mNormals[0]);
    vtkMath::Normalize(mNormals[2]);
    vtkMath::Normalize(mNormals[4]);
    for (i=0; i<3; i++)
    {
      mNormals[1][i] = -mNormals[0][i];
      mNormals[3][i] = -mNormals[2][i];
      mNormals[5][i] = -mNormals[4][i];
    }
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::GetPlanes(vtkPlanes *planes)
  {
    if ( ! planes )
    {
      return;
    }

    ComputeNormals();

    // Set the normals and coordinate values
    double factor = (InsideOut ? -1.0 : 1.0);
    for (int i=0; i<6; i++)
    {
      PlanePoints->SetPoint(i,Points->GetPoint(8+i));
      PlaneNormals->SetTuple3(i, factor*mNormals[i][0],
        factor*mNormals[i][1], factor*mNormals[i][2]);
    }

    planes->SetPoints(PlanePoints);
    planes->SetNormals(PlaneNormals);
    planes->Modified();
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::Rotate(int X,
    int Y,
    double *p1,
    double *p2,
    double *vpn)
  {
    double *pts =
      static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);
    double *center =
      static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(3*14);
    double v[3]; //vector of motion
    double axis[3]; //axis of rotation
    double theta; //rotation angle
    int i;

    v[0] = p2[0] - p1[0];
    v[1] = p2[1] - p1[1];
    v[2] = p2[2] - p1[2];

    // Create axis of rotation and angle of rotation
    vtkMath::Cross(vpn,v,axis);
    if ( vtkMath::Normalize(axis) == 0.0 )
    {
      return;
    }
    int *size = Renderer->GetSize();
    double l2 = (X-LastEventPosition[0])*(X-LastEventPosition[0])
      + (Y-LastEventPosition[1])*(Y-LastEventPosition[1]);
    theta = 360.0 * sqrt(l2/(size[0]*size[0]+size[1]*size[1]));

    //Manipulate the transform to reflect the rotation
    Transform->Identity();
    Transform->Translate(center[0],center[1],center[2]);
    Transform->RotateWXYZ(theta,axis);
    Transform->Translate(-center[0],-center[1],-center[2]);

    //Set the corners
    vtkPoints *newPts = vtkPoints::New(VTK_DOUBLE);
    Transform->TransformPoints(Points,newPts);

    for (i=0; i<8; i++, pts+=3)
    {
      Points->SetPoint(i, newPts->GetPoint(i));
    }

    newPts->Delete();
    PositionHandles();
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::CreateDefaultProperties()
  {
    // Current properties
    mHandleProperty = make_vtk<vtkProperty>();
    mSelectedHandleProperty = make_vtk<vtkProperty>();
    mHandleProperty->SetColor(0,0,0.9);
    mSelectedHandleProperty->SetColor(1,0,0);
    
    // Face properties
    mFaceProperty = make_vtk<vtkProperty>();
    mSelectedFaceProperty = make_vtk<vtkProperty>();
    mFaceProperty->SetColor(0,0,9);
    mFaceProperty->SetOpacity(0.0);
    mSelectedFaceProperty->SetColor(1,1,0);
    mSelectedFaceProperty->SetOpacity(0.25);

    // Outline properties
    mOutlineProperty = make_vtk<vtkProperty>();
    mSelectedOutlineProperty = make_vtk<vtkProperty>();;
    mOutlineProperty->SetRepresentationToWireframe();
    mOutlineProperty->SetAmbient(1.0);
    mOutlineProperty->SetAmbientColor(0,0,0.9);
    mOutlineProperty->SetLineWidth(1.0);
    mSelectedOutlineProperty->SetRepresentationToWireframe();
    mSelectedOutlineProperty->SetAmbient(1.0);
    mSelectedOutlineProperty->SetAmbientColor(0.0,1.0,0.0);
    mSelectedOutlineProperty->SetLineWidth(1.0);
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::PlaceWidget(double bds[6])
  {
    int i;
    double bounds[6], center[3];

    AdjustBounds(bds,bounds,center);

    Points->SetPoint(0, bounds[0], bounds[2], bounds[4]);
    Points->SetPoint(1, bounds[1], bounds[2], bounds[4]);
    Points->SetPoint(2, bounds[1], bounds[3], bounds[4]);
    Points->SetPoint(3, bounds[0], bounds[3], bounds[4]);
    Points->SetPoint(4, bounds[0], bounds[2], bounds[5]);
    Points->SetPoint(5, bounds[1], bounds[2], bounds[5]);
    Points->SetPoint(6, bounds[1], bounds[3], bounds[5]);
    Points->SetPoint(7, bounds[0], bounds[3], bounds[5]);

    for (i=0; i<6; i++)
    {
      InitialBounds[i] = bounds[i];
    }
    InitialLength = sqrt((bounds[1]-bounds[0])*(bounds[1]-bounds[0]) +
      (bounds[3]-bounds[2])*(bounds[3]-bounds[2]) +
      (bounds[5]-bounds[4])*(bounds[5]-bounds[4]));

    PositionHandles();
    ComputeNormals();
    ValidPick = 1; //since we have set up widget
    SizeHandles();
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::GetTransform(vtkTransform *t)
  {
    double *pts =
      static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);
    double *p0 = pts;
    double *p1 = pts + 3*1;
    double *p3 = pts + 3*3;
    double *p4 = pts + 3*4;
    double *p14 = pts + 3*14;
    double center[3], translate[3], scale[3], scaleVec[3][3];
    double InitialCenter[3];
    int i;

    // The transformation is relative to the initial bounds.
    // Initial bounds are set when PlaceWidget() is invoked.
    t->Identity();

    // Translation
    for (i=0; i<3; i++)
    {
      InitialCenter[i] =
        (InitialBounds[2*i+1]+InitialBounds[2*i]) / 2.0;
      center[i] = p14[i] - InitialCenter[i];
    }
    translate[0] = center[0] + InitialCenter[0];
    translate[1] = center[1] + InitialCenter[1];
    translate[2] = center[2] + InitialCenter[2];
    t->Translate(translate[0], translate[1], translate[2]);

    // Orientation
    Matrix->Identity();
    PositionHandles();
    ComputeNormals();
    for (i=0; i<3; i++)
    {
      Matrix->SetElement(i,0,mNormals[1][i]);
      Matrix->SetElement(i,1,mNormals[3][i]);
      Matrix->SetElement(i,2,mNormals[5][i]);
    }
    t->Concatenate(Matrix);

    // Scale
    for (i=0; i<3; i++)
    {
      scaleVec[0][i] = (p1[i] - p0[i]);
      scaleVec[1][i] = (p3[i] - p0[i]);
      scaleVec[2][i] = (p4[i] - p0[i]);
    }

    scale[0] = vtkMath::Norm(scaleVec[0]);
    if (InitialBounds[1] != InitialBounds[0])
    {
      scale[0] = scale[0] / (InitialBounds[1]-InitialBounds[0]);
    }
    scale[1] = vtkMath::Norm(scaleVec[1]);
    if (InitialBounds[3] != InitialBounds[2])
    {
      scale[1] = scale[1] / (InitialBounds[3]-InitialBounds[2]);
    }
    scale[2] = vtkMath::Norm(scaleVec[2]);
    if (InitialBounds[5] != InitialBounds[4])
    {
      scale[2] = scale[2] / (InitialBounds[5]-InitialBounds[4]);
    }
    t->Scale(scale[0],scale[1],scale[2]);

    // Add back in the contribution due to non-origin center
    t->Translate(-InitialCenter[0], -InitialCenter[1], -InitialCenter[2]);
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::SetTransform(vtkTransform* t)
  {
    if (!t)
    {
      vtkErrorMacro(<<"vtkTransform t must be non-nullptr");
      return;
    }

    double *pts =
      static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);
    double xIn[3];
    // make sure the transform is up-to-date before using it
    t->Update();

    // Position the eight points of the box and then update the
    // position of the other handles.
    double *bounds=InitialBounds;

    xIn[0] = bounds[0]; xIn[1] = bounds[2]; xIn[2] = bounds[4];
    t->InternalTransformPoint(xIn,pts);

    xIn[0] = bounds[1]; xIn[1]= bounds[2]; xIn[2] = bounds[4];
    t->InternalTransformPoint(xIn,pts+3);

    xIn[0] = bounds[1]; xIn[1]= bounds[3]; xIn[2] = bounds[4];
    t->InternalTransformPoint(xIn,pts+6);

    xIn[0] = bounds[0]; xIn[1]= bounds[3]; xIn[2] = bounds[4];
    t->InternalTransformPoint(xIn,pts+9);

    xIn[0] = bounds[0]; xIn[1]= bounds[2]; xIn[2] = bounds[5];
    t->InternalTransformPoint(xIn,pts+12);

    xIn[0] = bounds[1]; xIn[1]= bounds[2]; xIn[2] = bounds[5];
    t->InternalTransformPoint(xIn,pts+15);

    xIn[0] = bounds[1]; xIn[1]= bounds[3]; xIn[2] = bounds[5];
    t->InternalTransformPoint(xIn,pts+18);

    xIn[0] = bounds[0]; xIn[1]= bounds[3]; xIn[2] = bounds[5];
    t->InternalTransformPoint(xIn,pts+21);

    PositionHandles();
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::SetOutlineFaceWires(int newValue)
  {
    if (OutlineFaceWires != newValue)
    {
      OutlineFaceWires = newValue;
      Modified();
      // the outline is dependent on this value, so we have to regen
      GenerateOutline();
    }
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::SetOutlineCursorWires(int newValue)
  {
    if (OutlineCursorWires != newValue)
    {
      OutlineCursorWires = newValue;
      Modified();
      // the outline is dependent on this value, so we have to regen
      GenerateOutline();
    }
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::GenerateOutline()
  {
    // Now the outline lines
    if ( ! OutlineFaceWires && ! OutlineCursorWires )
    {
      return;
    }
    for(int i(0); i<mFaces.size(); ++i)
    {
      mFaces[i].outlineData->Modified();
    }
  }

  //----------------------------------------------------------------------------
  int ROIRepresentation::ComputeInteractionState(int X, int Y, int modify)
  {
    // Okay, we can process this. Try to pick handles first;
    // if no handles picked, then pick the bounding box.
    if (!Renderer || !Renderer->IsInViewport(X, Y))
    {
      InteractionState = ROIRepresentation::Outside;
      return InteractionState;
    }

    ComputeNormals();

    // Try and pick a handle first
    LastPicker = nullptr;
    CurrentHandle = nullptr;

    vtkAssemblyPath* path = GetAssemblyPath(X, Y, 0., HandlePicker);

    if ( path != nullptr )
    {
      ValidPick = 1;
      LastPicker = HandlePicker;
      CurrentHandle = reinterpret_cast<vtkActor *>(path->GetFirstNode()->GetViewProp());
      if ( CurrentHandle == mFaces[0].outlineActor)
      {
        InteractionState = ROIRepresentation::MoveF0;
      }
      else if ( CurrentHandle == mFaces[1].outlineActor)
      {
        InteractionState = ROIRepresentation::MoveF1;
      }
      else if ( CurrentHandle == mFaces[2].outlineActor)
      {
        InteractionState = ROIRepresentation::MoveF2;
      }
      else if ( CurrentHandle == mFaces[3].outlineActor)
      {
        InteractionState = ROIRepresentation::MoveF3;
      }
      else if ( CurrentHandle == mFaces[4].outlineActor)
      {
        InteractionState = ROIRepresentation::MoveF4;
      }
      else if ( CurrentHandle == mFaces[5].outlineActor)
      {
        InteractionState = ROIRepresentation::MoveF5;
      }
    }
    else //see if the hex is picked
    {
      path = GetAssemblyPath(X, Y, 0., HexPicker);

      if ( path != nullptr )
      {
        LastPicker = HexPicker;
        ValidPick = 1;
        if ( !modify )
        {
          InteractionState = ROIRepresentation::Rotating;
        }
      }
      else
      {
        InteractionState = ROIRepresentation::Outside;
      }
    }

    return InteractionState;
  }

  //----------------------------------------------------------------------
  void ROIRepresentation::SetInteractionState(int state)
  {
    // Clamp to allowable values
    state = ( state < ROIRepresentation::Outside ? ROIRepresentation::Outside :
      (state > ROIRepresentation::Scaling ? ROIRepresentation::Scaling : state) );

    // Depending on state, highlight appropriate parts of representation
    int handle;
    InteractionState = state;
    switch (state)
    {
    case ROIRepresentation::MoveF0:
    case ROIRepresentation::MoveF1:
    case ROIRepresentation::MoveF2:
    case ROIRepresentation::MoveF3:
    case ROIRepresentation::MoveF4:
    case ROIRepresentation::MoveF5:
      HighlightOutline(1);
      handle = HighlightHandle(CurrentHandle);
      HighlightFace(handle);
      break;
    case ROIRepresentation::Rotating:
      HighlightOutline(0);
      HighlightHandle(nullptr);
      HighlightFace(HexPicker->GetCellId());
      break;
    case ROIRepresentation::Translating:
    case ROIRepresentation::Scaling:
      HighlightOutline(1);
      HighlightFace(-1);
      break;
    default:
      HighlightOutline(0);
      HighlightFace(-1);
    }
  }

  //----------------------------------------------------------------------
  double* ROIRepresentation::GetBounds()
  {
    BuildRepresentation();
    BoundingBox->SetBounds(mFaces[0].faceActor->GetBounds());
    for (int i(1); i < mFaces.size(); ++i)
      BoundingBox->AddBounds(mFaces[i].faceActor->GetBounds());
    return BoundingBox->GetBounds();
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::BuildRepresentation()
  {
    // Rebuild only if necessary
    if ( GetMTime() > BuildTime ||
      (Renderer && Renderer->GetVTKWindow() &&
      (Renderer->GetVTKWindow()->GetMTime() > BuildTime ||
        Renderer->GetActiveCamera()->GetMTime() > BuildTime)) )
    {
      SizeHandles();
      BuildTime.Modified();
    }
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::updateVisibility(double position[3])
  {
    double min[3];
    double max[3];
    
    BoundingBox->GetXMin(min);
    BoundingBox->GetXMax(max);
    auto* camera = Renderer->GetActiveCamera();
    double pos[3];
    double focalPoint[3];
    camera->GetPosition(pos);
    camera->GetFocalPoint(focalPoint);
    
    vtkVector3d v;
    for (int i(0); i < 3; ++i)
      v[i] = pos[i] - focalPoint[i];
    v.Normalize();

    bool isInside = true;
    for (int i(0); i<3; ++i)
    {
      if (std::abs(1-std::abs(v[i])) < 10e-5)
        if (max[i] < position[i] || min[i] > position[i])
          isInside = false;
    }
    for (int j=0; j<mFaces.size(); ++j)
    {
      mFaces[j].outlineActor->SetVisibility(isInside);
      mFaces[j].faceActor->SetVisibility(isInside);
    }
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::ReleaseGraphicsResources(vtkWindow *w)
  {
    for (int j=0; j<mFaces.size(); j++)
    {
      mFaces[j].faceActor->ReleaseGraphicsResources(w);
      mFaces[j].outlineActor->ReleaseGraphicsResources(w);
    }
  }

  //----------------------------------------------------------------------------
  int ROIRepresentation::RenderOpaqueGeometry(vtkViewport *v)
  {
    int count=0;
    BuildRepresentation();
    for (int j=0; j<mFaces.size(); j++)
    {
      if (mFaces[j].faceActor->GetVisibility())
      {
        count += mFaces[j].faceActor->RenderOpaqueGeometry(v);
        count += mFaces[j].outlineActor->RenderOpaqueGeometry(v);
      }
    }
    return count;
  }

  //----------------------------------------------------------------------------
  int ROIRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport *v)
  {
    int count=0;
    BuildRepresentation();
    for (int j=0; j<mFaces.size(); j++)
    {
      if (mFaces[j].faceActor->GetVisibility())
      {
        count += mFaces[j].faceActor->RenderTranslucentPolygonalGeometry(v);
        count += mFaces[j].outlineActor->RenderTranslucentPolygonalGeometry(v);
      }
    }
    return count;
  }

  //----------------------------------------------------------------------------
  int ROIRepresentation::HasTranslucentPolygonalGeometry()
  {
    int result=0;
    BuildRepresentation();
    for (int j=0; j<mFaces.size(); j++)
    {
      if (mFaces[j].faceActor->GetVisibility())
      {
        result |= mFaces[j].faceActor->HasTranslucentPolygonalGeometry();
        result |= mFaces[j].outlineActor->HasTranslucentPolygonalGeometry();
      }
    }
    return result;
  }

  #define VTK_AVERAGE(a,b,c) \
    c[0] = (a[0] + b[0])/2.0; \
    c[1] = (a[1] + b[1])/2.0; \
    c[2] = (a[2] + b[2])/2.0;

  //----------------------------------------------------------------------------
  void ROIRepresentation::PositionHandles()
  {
    double *pts =
      static_cast<vtkDoubleArray *>(Points->GetData())->GetPointer(0);
    double *p0 = pts;
    double *p1 = pts + 3*1;
    double *p2 = pts + 3*2;
    double *p3 = pts + 3*3;
    //double *p4 = pts + 3*4;
    double *p5 = pts + 3*5;
    double *p6 = pts + 3*6;
    double *p7 = pts + 3*7;
    double x[3];

    VTK_AVERAGE(p0,p7,x);
    Points->SetPoint(8, x);
    VTK_AVERAGE(p1,p6,x);
    Points->SetPoint(9, x);
    VTK_AVERAGE(p0,p5,x);
    Points->SetPoint(10, x);
    VTK_AVERAGE(p2,p7,x);
    Points->SetPoint(11, x);
    VTK_AVERAGE(p1,p3,x);
    Points->SetPoint(12, x);
    VTK_AVERAGE(p5,p7,x);
    Points->SetPoint(13, x);
    VTK_AVERAGE(p0,p6,x);
    Points->SetPoint(14, x);

    Points->GetData()->Modified();
    GenerateOutline();
  }
  #undef VTK_AVERAGE

  //----------------------------------------------------------------------------
  void ROIRepresentation::HandlesOn()
  {
    for (int i=0; i<mFaces.size(); ++i)
    {
      mFaces[i].faceActor->VisibilityOn();
      mFaces[i].outlineActor->VisibilityOn();
    }
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::HandlesOff()
  {
    for (int i=0; i<mFaces.size(); ++i)
    {
      mFaces[i].faceActor->VisibilityOff();
      mFaces[i].outlineActor->VisibilityOff();
    }
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::SizeHandles()
  {
    double *center = static_cast<vtkDoubleArray*>(Points->GetData())->GetPointer(3*14);
    double radius  = vtkWidgetRepresentation::SizeHandlesInPixels(1.5,center);
  }

  //----------------------------------------------------------------------------
  int ROIRepresentation::HighlightHandle(vtkProp *prop)
  {
    // first unhighlight anything picked
    HighlightOutline(0);
    if ( CurrentHandle )
    {
      CurrentHandle->SetProperty(mHandleProperty);
    }

    CurrentHandle = static_cast<vtkActor *>(prop);
    if ( CurrentHandle )
    {
      CurrentHandle->SetProperty(mSelectedHandleProperty);
      for (int i=0; i<mFaces.size(); ++i) //find attached face
      {
        if ( CurrentHandle == mFaces[i].outlineActor )
        {
          return i;
        }
      }
    }
    return -1;
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::HighlightFace(int cellId)
  {
    if ( cellId >= 0 && cellId < mFaces.size() )
    {
      mFaces[cellId].outlineActor->SetProperty(mSelectedOutlineProperty);
      if ( ! CurrentHandle )
      {
        CurrentHandle = mFaces[cellId].outlineActor;
      }
    }
    else
    {
      CurrentHexFace = -1;
    }
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::HighlightOutline(int highlight)
  {
    for (int i(0); i<mFaces.size(); ++i)
    {
      if ( highlight )
      {
        mFaces[i].outlineActor->SetProperty(mSelectedOutlineProperty);
      }
      else
      {
        mFaces[i].outlineActor->SetProperty(mOutlineProperty);
      }
    }
  }

  //------------------------------------------------------------------------------
  void ROIRepresentation::RegisterPickers()
  {
    Renderer->GetRenderWindow()->GetInteractor()->GetPickingManager()->AddPicker(HandlePicker, this);
    Renderer->GetRenderWindow()->GetInteractor()->GetPickingManager()->AddPicker(HexPicker, this);
  }

  //----------------------------------------------------------------------------
  void ROIRepresentation::PrintSelf(ostream& os, vtkIndent indent)
  {
    Superclass::PrintSelf(os,indent);

    double *bounds=InitialBounds;
    os << indent << "Initial Bounds: "
      << "(" << bounds[0] << "," << bounds[1] << ") "
      << "(" << bounds[2] << "," << bounds[3] << ") "
      << "(" << bounds[4] << "," << bounds[5] << ")\n";

    if ( mHandleProperty )
    {
      os << indent << "Handle Property: " << mHandleProperty << "\n";
    }
    else
    {
      os << indent << "Handle Property: (none)\n";
    }
    if ( mSelectedHandleProperty )
    {
      os << indent << "Selected Handle Property: "
        << mSelectedHandleProperty << "\n";
    }
    else
    {
      os << indent << "SelectedHandle Property: (none)\n";
    }

    if ( mFaceProperty )
    {
      os << indent << "Face Property: " <<mFaceProperty << "\n";
    }
    else
    {
      os << indent << "Face Property: (none)\n";
    }
    if ( mSelectedFaceProperty )
    {
      os << indent << "Selected Face Property: "
        << mSelectedFaceProperty << "\n";
    }
    else
    {
      os << indent << "Selected Face Property: (none)\n";
    }

    if ( mOutlineProperty )
    {
      os << indent << "Outline Property: " << mOutlineProperty << "\n";
    }
    else
    {
      os << indent << "Outline Property: (none)\n";
    }
    if ( mSelectedOutlineProperty )
    {
      os << indent << "Selected Outline Property: "
        << mSelectedOutlineProperty << "\n";
    }
    else
    {
      os << indent << "Selected Outline Property: (none)\n";
    }

    os << indent << "Outline Face Wires: "
      << (OutlineFaceWires ? "On\n" : "Off\n");
    os << indent << "Outline Cursor Wires: "
      << (OutlineCursorWires ? "On\n" : "Off\n");
    os << indent << "Inside Out: " << (InsideOut ? "On\n" : "Off\n");
  }
}
}