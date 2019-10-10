#include "private/muk.pch"
#include "MukVtkAxesActor.h"

#include "MukCommon/vtk_tools.h"

#include "vtkActor.h"
#include "vtkConeSource.h"
#include "vtkCylinderSource.h"
#include "vtkLineSource.h"
#include "vtkObject.h"
#include "vtkObjectFactory.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkPropCollection.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkSphereSource.h"
#include "vtkTransform.h"

#include <algorithm>

namespace
{
  using namespace gris::muk;
}

vtkStandardNewMacro(MukVtkAxesActor);

//----------------------------------------------------------------------------
MukVtkAxesActor::MukVtkAxesActor()
{
  mpXAxisShaft = make_vtk<vtkActor>();
  mpYAxisShaft = make_vtk<vtkActor>();
  mpZAxisShaft = make_vtk<vtkActor>();
  mpXAxisShaft->GetProperty()->SetColor(1, 0, 0);
  mpYAxisShaft->GetProperty()->SetColor(0, 1, 0);
  mpZAxisShaft->GetProperty()->SetColor(0, 0, 1);

  mpXAxisTip = make_vtk<vtkActor>();
  mpYAxisTip = make_vtk<vtkActor>();
  mpZAxisTip = make_vtk<vtkActor>();
  mpXAxisTip->GetProperty()->SetColor(1, 0, 0);
  mpYAxisTip->GetProperty()->SetColor(0, 1, 0);
  mpZAxisTip->GetProperty()->SetColor(0, 0, 1);

  mpCylinderSource = make_vtk<vtkCylinderSource>();
  mpCylinderSource->SetHeight(1.0);

  mpLineSource = make_vtk<vtkLineSource>();
  mpLineSource->SetPoint1( 0.0, 0.0, 0.0 );
  mpLineSource->SetPoint2( 0.0, 1.0, 0.0 );

  mpConeSource = make_vtk<vtkConeSource>();
  mpConeSource->SetDirection( 0, 1, 0 );
  mpConeSource->SetHeight( 1.0 );

  mpSphereSource = make_vtk<vtkSphereSource>();
  {
    auto shaftMapper = make_vtk<vtkPolyDataMapper>();
    mpXAxisShaft->SetMapper( shaftMapper );
    mpYAxisShaft->SetMapper( shaftMapper );
    mpZAxisShaft->SetMapper( shaftMapper );
    auto tipMapper = make_vtk<vtkPolyDataMapper>();
    mpXAxisTip->SetMapper( tipMapper );
    mpYAxisTip->SetMapper( tipMapper );
    mpZAxisTip->SetMapper( tipMapper );
  }

  mTotalLength[0] = 1.0;
  mTotalLength[1] = 1.0;
  mTotalLength[2] = 1.0;

  mNormalizedShaftLength[0] = 0.8;
  mNormalizedShaftLength[1] = 0.8;
  mNormalizedShaftLength[2] = 0.8;

  mNormalizedTipLength[0] = 0.2;
  mNormalizedTipLength[1] = 0.2;
  mNormalizedTipLength[2] = 0.2;

  mNormalizedLabelPosition[0] = 1.0;
  mNormalizedLabelPosition[1] = 1.0;
  mNormalizedLabelPosition[2] = 1.0;

  mConeResolution = 16;
  mSphereResolution = 16;
  mCylinderResolution = 16;

  mConeRadius = 0.4;
  mSphereRadius = 0.5;
  mCylinderRadius = 0.05;

  mShaftType = MukVtkAxesActor::LINE_SHAFT;
  mTipType   = MukVtkAxesActor::CONE_TIP;
  
  UpdateProps();
}

//----------------------------------------------------------------------------
MukVtkAxesActor::~MukVtkAxesActor()
{
}

//----------------------------------------------------------------------------
/**
* Shallow copy of an axes actor. Overloads the virtual vtkProp method.
*/
void MukVtkAxesActor::ShallowCopy(vtkProp* prop)
{
  MukVtkAxesActor* a = MukVtkAxesActor::SafeDownCast(prop);
  if ( a != nullptr )
  {
    SetTotalLength( a->GetmTotalLength() );
    SetNormalizedShaftLength( a->GetmNormalizedShaftLength() );
    SetNormalizedTipLength( a->GetmNormalizedTipLength() );
    SetNormalizedLabelPosition( a->GetmNormalizedLabelPosition() );
    SetmConeResolution( a->GetmConeResolution() );
    SetmSphereResolution( a->GetmSphereResolution() );
    SetmCylinderResolution( a->GetmCylinderResolution() );
    SetmConeRadius( a->GetmConeRadius() );
    SetmSphereRadius( a->GetmSphereRadius() );
    SetmCylinderRadius( a->GetmCylinderRadius() );
    SetTipType( a->GetmTipType() );
    SetShaftType( a->GetmShaftType() );
  }

  // Now do superclass
  vtkProp3D::ShallowCopy(prop);
}

//----------------------------------------------------------------------------
/**
* For some exporters and other other operations we must be
* able to collect all the actors or volumes. These methods
* are used in that process.
*/
void MukVtkAxesActor::GetActors(vtkPropCollection* ac)
{
  ac->AddItem( mpXAxisShaft );
  ac->AddItem( mpYAxisShaft );
  ac->AddItem( mpZAxisShaft );
  ac->AddItem( mpXAxisTip );
  ac->AddItem( mpYAxisTip );
  ac->AddItem( mpZAxisTip );
}

//----------------------------------------------------------------------------
//@{
/**
* Support the standard render methods.
*/
int MukVtkAxesActor::RenderOpaqueGeometry(vtkViewport* vp)
{
  UpdateProps();
  int renderedSomething = 0;
  renderedSomething += mpXAxisShaft->RenderOpaqueGeometry( vp );
  renderedSomething += mpYAxisShaft->RenderOpaqueGeometry( vp );
  renderedSomething += mpZAxisShaft->RenderOpaqueGeometry( vp );
  renderedSomething += mpXAxisTip->RenderOpaqueGeometry( vp );
  renderedSomething += mpYAxisTip->RenderOpaqueGeometry( vp );
  renderedSomething += mpZAxisTip->RenderOpaqueGeometry( vp );
  renderedSomething = (renderedSomething > 0)?(1):(0);
  return renderedSomething;
}

//-----------------------------------------------------------------------------
//@{
/**
* Support the standard render methods.
*/
int MukVtkAxesActor::RenderTranslucentPolygonalGeometry(vtkViewport* vp)
{
  UpdateProps();
  int renderedSomething = 0;
  renderedSomething += mpXAxisShaft->RenderTranslucentPolygonalGeometry( vp );
  renderedSomething += mpYAxisShaft->RenderTranslucentPolygonalGeometry( vp );
  renderedSomething += mpZAxisShaft->RenderTranslucentPolygonalGeometry( vp );
  renderedSomething += mpXAxisTip->RenderTranslucentPolygonalGeometry( vp );
  renderedSomething += mpYAxisTip->RenderTranslucentPolygonalGeometry( vp );
  renderedSomething += mpZAxisTip->RenderTranslucentPolygonalGeometry( vp );
  renderedSomething = (renderedSomething > 0)?(1):(0);
  return renderedSomething;
}

//-----------------------------------------------------------------------------
/*
  Does this prop have some translucent polygonal geometry?
*/
int MukVtkAxesActor::HasTranslucentPolygonalGeometry()
{
  UpdateProps();
  int result = 0;
  result |= mpXAxisShaft->HasTranslucentPolygonalGeometry();
  result |= mpYAxisShaft->HasTranslucentPolygonalGeometry();
  result |= mpZAxisShaft->HasTranslucentPolygonalGeometry();
  result |= mpXAxisTip->HasTranslucentPolygonalGeometry();
  result |= mpYAxisTip->HasTranslucentPolygonalGeometry();
  result |= mpZAxisTip->HasTranslucentPolygonalGeometry();
  return result;
}

//----------------------------------------------------------------------------
/**
* Release any graphics resources that are being consumed by this actor.
* The parameter window could be used to determine which graphic
* resources to release.
*/
void MukVtkAxesActor::ReleaseGraphicsResources(vtkWindow* win)
{
  mpXAxisShaft->ReleaseGraphicsResources( win );
  mpYAxisShaft->ReleaseGraphicsResources( win );
  mpZAxisShaft->ReleaseGraphicsResources( win );
  mpXAxisTip->ReleaseGraphicsResources( win );
  mpYAxisTip->ReleaseGraphicsResources( win );
  mpZAxisTip->ReleaseGraphicsResources( win );
}

//----------------------------------------------------------------------------
//@{
/**
* Get the bounds for this Actor as (Xmin,Xmax,Ymin,Ymax,Zmin,Zmax). (The
* method GetBounds(double bounds[6]) is available from the superclass.)
*/
void MukVtkAxesActor::GetBounds(double bounds[6])
{
  double* bds = GetBounds();
  bounds[0] = bds[0];
  bounds[1] = bds[1];
  bounds[2] = bds[2];
  bounds[3] = bds[3];
  bounds[4] = bds[4];
  bounds[5] = bds[5];
}

//----------------------------------------------------------------------------
// Get the bounds for this Actor as (Xmin,Xmax,Ymin,Ymax,Zmin,Zmax).
double* MukVtkAxesActor::GetBounds()
{
  mpXAxisShaft->GetBounds(Bounds);
  double boundsX[6];
  double boundsY[6];
  double boundsZ[6];
  int i;

  mpYAxisShaft->GetBounds(boundsY);
  mpZAxisShaft->GetBounds(boundsZ);
  for ( i = 0; i < 3; ++i )
  {
    Bounds[2*i+1] = std::max(boundsY[2*i+1], Bounds[2*i+1]);
    Bounds[2*i+1] = std::max(boundsZ[2*i+1], Bounds[2*i+1]);
    Bounds[2*i]   = std::min(boundsY[2*i],   Bounds[2*i]);
    Bounds[2*i]   = std::min(boundsZ[2*i],   Bounds[2*i]);
  }
  mpXAxisTip->GetBounds(boundsX);
  mpYAxisTip->GetBounds(boundsY);
  mpZAxisTip->GetBounds(boundsZ);
  for ( i = 0; i < 3; ++i )
  {
    Bounds[2*i+1] = std::max(boundsX[2*i+1], Bounds[2*i+1]);
    Bounds[2*i+1] = std::max(boundsY[2*i+1], Bounds[2*i+1]);
    Bounds[2*i+1] = std::max(boundsZ[2*i+1], Bounds[2*i+1]);
    Bounds[2*i]   = std::min(boundsX[2*i],   Bounds[2*i]);
    Bounds[2*i]   = std::min(boundsY[2*i],   Bounds[2*i]);
    Bounds[2*i]   = std::min(boundsZ[2*i],   Bounds[2*i]);
  }

  /*(vtkPolyDataMapper::SafeDownCast(mpYAxisShaft->GetMapper()))->GetInput()->GetBounds( boundsY );
  for ( i = 0; i < 3; ++i )
  {
    Bounds[2*i+1] = std::max(boundsY[2*i+1], Bounds[2*i+1]);
    Bounds[2*i]   = std::min(boundsY[2*i],   Bounds[2*i]);
  }*/

  return Bounds;
}

//----------------------------------------------------------------------------
//@}

/**
* Get the actors mtime plus consider its properties and texture if set.
*/
vtkMTimeType MukVtkAxesActor::GetMTime()
{
  vtkMTimeType mTime = Superclass::GetMTime();
  return mTime;
}

//----------------------------------------------------------------------------
/**
* Return the mtime of anything that would cause the rendered image to
* appear differently. Usually this involves checking the mtime of the
* prop plus anything else it depends on such as properties, textures
* etc.
*/
vtkMTimeType MukVtkAxesActor::GetRedrawMTime()
{
  vtkMTimeType mTime = GetMTime();
  return mTime;
}

//----------------------------------------------------------------------------
//@{
/**
* Set the total length of the axes in 3 dimensions.
*/
void MukVtkAxesActor::SetTotalLength( double x, double y, double z )
{
  if ( mTotalLength[0] != x ||    mTotalLength[1] != y ||    mTotalLength[2] != z )
  {
    mTotalLength[0] = x;
    mTotalLength[1] = y;
    mTotalLength[2] = z;
    if ( x < 0.0 || y < 0.0 || z < 0.0 )
    {
      vtkGenericWarningMacro("One or more axes lengths are < 0 \
                        and may produce unexpected results.");
    }
    Modified();
    UpdateProps();
  }
}

//----------------------------------------------------------------------------
/**
* Set the normalized (0-1) length of the shaft.
*/
void MukVtkAxesActor::SetNormalizedShaftLength( double x, double y, double z )
{
  if ( mNormalizedShaftLength[0] != x ||
    mNormalizedShaftLength[1] != y ||
    mNormalizedShaftLength[2] != z )
  {
    mNormalizedShaftLength[0] = x;
    mNormalizedShaftLength[1] = y;
    mNormalizedShaftLength[2] = z;
    if ( x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0 || z < 0.0 || z > 1.0 )
    {
      vtkGenericWarningMacro( "One or more normalized shaft lengths \
      are < 0 or > 1 and may produce unexpected results." );
    }
    Modified();
    UpdateProps();
  }
}

//---------------------------------------------------------------------------- 
/**
  * Set the normalized (0-1) length of the tip.  Normally, this would be
  * 1 - the normalized length of the shaft.
*/
void MukVtkAxesActor::SetNormalizedTipLength( double x, double y, double z )
{
  if ( mNormalizedTipLength[0] != x ||
    mNormalizedTipLength[1] != y ||
    mNormalizedTipLength[2] != z )
  {
    mNormalizedTipLength[0] = x;
    mNormalizedTipLength[1] = y;
    mNormalizedTipLength[2] = z;
    if ( x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0 || z < 0.0 || z > 1.0 )
    {
      vtkGenericWarningMacro( "One or more normalized tip lengths are < 0 or > 1 and may produce unexpected results." );
    }
    Modified();
    UpdateProps();
  }
}

//----------------------------------------------------------------------------
/**
* Set the normalized (0-1) position of the label along the length of
* the shaft.  A value > 1 is permissible.
*/
void MukVtkAxesActor::SetNormalizedLabelPosition( double x, double y, double z )
{
  if ( mNormalizedLabelPosition[0] != x ||
    mNormalizedLabelPosition[1] != y ||
    mNormalizedLabelPosition[2] != z )
  {
    mNormalizedLabelPosition[0] = x;
    mNormalizedLabelPosition[1] = y;
    mNormalizedLabelPosition[2] = z;
    if ( x < 0.0 || y < 0.0 || z < 0.0 )
    {
      vtkGenericWarningMacro( "One or more label positions are < 0 and may produce unexpected results." );
    }
    Modified();
    UpdateProps();
  }
}

//----------------------------------------------------------------------------
void MukVtkAxesActor::SetShaftType( int type )
{
  if ( mShaftType != type )
  {
    if (type < MukVtkAxesActor::CYLINDER_SHAFT)
    {
      vtkErrorMacro( "Undefined axes shaft type." );
      return;
    }
    mShaftType = type;
    Modified();
    UpdateProps();
  }
}

//----------------------------------------------------------------------------
void MukVtkAxesActor::SetTipType( int type )
{
  if ( mTipType != type )
  {
    if (type < MukVtkAxesActor::CONE_TIP)
    {
      vtkErrorMacro( "Undefined axes tip type." );
      return;
    }
    mTipType = type;
    Modified();
    UpdateProps();
  }
}

//----------------------------------------------------------------------------
void MukVtkAxesActor::UpdateProps()
{
  mpCylinderSource->SetRadius( mCylinderRadius );
  mpCylinderSource->SetResolution( mCylinderResolution );
  mpConeSource->SetResolution( mConeResolution );
  mpConeSource->SetRadius( mConeRadius );
  mpSphereSource->SetThetaResolution( mSphereResolution );
  mpSphereSource->SetPhiResolution( mSphereResolution );
  mpSphereSource->SetRadius( mSphereRadius );

  switch ( mShaftType )
  {
    case MukVtkAxesActor::CYLINDER_SHAFT:
      (vtkPolyDataMapper::SafeDownCast(mpXAxisShaft->GetMapper()))->SetInputConnection( mpCylinderSource->GetOutputPort() );
      break;
    case MukVtkAxesActor::LINE_SHAFT:
      (vtkPolyDataMapper::SafeDownCast(mpXAxisShaft->GetMapper()))->SetInputConnection( mpLineSource->GetOutputPort() );
      break;
  }

  switch ( mTipType )
  {
    case MukVtkAxesActor::CONE_TIP:
      (vtkPolyDataMapper::SafeDownCast(mpXAxisTip->GetMapper()))->SetInputConnection( mpConeSource->GetOutputPort() );
      break;
    case MukVtkAxesActor::SPHERE_TIP:
      (vtkPolyDataMapper::SafeDownCast(mpXAxisTip->GetMapper()))->SetInputConnection( mpSphereSource->GetOutputPort() );
      break;
  }

  vtkPolyDataMapper::SafeDownCast(mpXAxisTip->GetMapper())->GetInputAlgorithm()->Update();
  vtkPolyDataMapper::SafeDownCast(mpXAxisShaft->GetMapper())->GetInputAlgorithm()->Update();

  if ( GetUserTransform() )
  {
    mpXAxisShaft->SetUserTransform( nullptr );
    mpYAxisShaft->SetUserTransform( nullptr );
    mpZAxisShaft->SetUserTransform( nullptr );
    mpXAxisTip->SetUserTransform( nullptr );
    mpYAxisTip->SetUserTransform( nullptr );
    mpZAxisTip->SetUserTransform( nullptr );
  }

  // The shaft and tip geometry are both initially along direction 0 1 0
  // in the case of cylinder, line, and cone.  Build up the axis from
  // constituent elements defined in their default positions.

  double bounds[6];
  vtkPolyDataMapper::SafeDownCast(mpXAxisShaft->GetMapper())->GetInput()->GetBounds( bounds );
  double scale[3];
  int i;
  for ( i = 0; i < 3; ++i )
  {
    scale[i] = mNormalizedShaftLength[i]*mTotalLength[i] / (bounds[3] - bounds[2]);
  }

  auto xTransform = make_vtk<vtkTransform>();
  auto yTransform = make_vtk<vtkTransform>();
  auto zTransform = make_vtk<vtkTransform>();

  xTransform->RotateZ( -90 );
  zTransform->RotateX( 90 );

  xTransform->Scale( scale[0], scale[0], scale[0] );
  yTransform->Scale( scale[1], scale[1], scale[1] );
  zTransform->Scale( scale[2], scale[2], scale[2] );

  xTransform->Translate( -(bounds[0]+bounds[1])/2,
    -bounds[2],
    -(bounds[4]+bounds[5])/2 );
  yTransform->Translate( -(bounds[0]+bounds[1])/2,
    -bounds[2],
    -(bounds[4]+bounds[5])/2 );
  zTransform->Translate( -(bounds[0]+bounds[1])/2,
    -bounds[2],
    -(bounds[4]+bounds[5])/2 );

  mpXAxisShaft->SetScale( xTransform->GetScale() );
  mpXAxisShaft->SetPosition( xTransform->GetPosition() );
  mpXAxisShaft->SetOrientation( xTransform->GetOrientation() );

  mpYAxisShaft->SetScale( yTransform->GetScale() );
  mpYAxisShaft->SetPosition( yTransform->GetPosition() );
  mpYAxisShaft->SetOrientation( yTransform->GetOrientation() );

  mpZAxisShaft->SetScale( zTransform->GetScale() );
  mpZAxisShaft->SetPosition( zTransform->GetPosition() );
  mpZAxisShaft->SetOrientation( zTransform->GetOrientation() );

  (vtkPolyDataMapper::SafeDownCast(mpXAxisTip->GetMapper()))->
    GetInput()->GetBounds( bounds );

  xTransform->Identity();
  yTransform->Identity();
  zTransform->Identity();

  xTransform->RotateZ( -90 );
  zTransform->RotateX( 90 );

  xTransform->Scale( mTotalLength[0], mTotalLength[0], mTotalLength[0] );
  yTransform->Scale( mTotalLength[1], mTotalLength[1], mTotalLength[1] );
  zTransform->Scale( mTotalLength[2], mTotalLength[2], mTotalLength[2] );

  xTransform->Translate( 0, (1.0 - mNormalizedTipLength[0]), 0 );
  yTransform->Translate( 0, (1.0 - mNormalizedTipLength[1]), 0 );
  zTransform->Translate( 0, (1.0 - mNormalizedTipLength[2]), 0 );

  xTransform->Scale( mNormalizedTipLength[0],
    mNormalizedTipLength[0],
    mNormalizedTipLength[0] );

  yTransform->Scale( mNormalizedTipLength[1],
    mNormalizedTipLength[1],
    mNormalizedTipLength[1] );

  zTransform->Scale( mNormalizedTipLength[2],
    mNormalizedTipLength[2],
    mNormalizedTipLength[2] );

  xTransform->Translate( -(bounds[0]+bounds[1])/2,
    -bounds[2],
    -(bounds[4]+bounds[5])/2 );
  yTransform->Translate( -(bounds[0]+bounds[1])/2,
    -bounds[2],
    -(bounds[4]+bounds[5])/2 );
  zTransform->Translate( -(bounds[0]+bounds[1])/2,
    -bounds[2],
    -(bounds[4]+bounds[5])/2 );

  mpXAxisTip->SetScale( xTransform->GetScale() );
  mpXAxisTip->SetPosition( xTransform->GetPosition() );
  mpXAxisTip->SetOrientation( xTransform->GetOrientation() );

  mpYAxisTip->SetScale( yTransform->GetScale() );
  mpYAxisTip->SetPosition( yTransform->GetPosition() );
  mpYAxisTip->SetOrientation( yTransform->GetOrientation() );

  mpZAxisTip->SetScale( zTransform->GetScale() );
  mpZAxisTip->SetPosition( zTransform->GetPosition() );
  mpZAxisTip->SetOrientation( zTransform->GetOrientation() );
    
  vtkLinearTransform* transform = GetUserTransform();
  if ( transform )
  {
    mpXAxisShaft->SetUserTransform( transform );
    mpYAxisShaft->SetUserTransform( transform );
    mpZAxisShaft->SetUserTransform( transform );

    mpXAxisTip->SetUserTransform( transform );
    mpYAxisTip->SetUserTransform( transform );
    mpZAxisTip->SetUserTransform( transform );
  }
}

//----------------------------------------------------------------------------
vtkProperty* MukVtkAxesActor::GetXAxisTipProperty()
{
  return mpXAxisTip->GetProperty();
}

//----------------------------------------------------------------------------
vtkProperty* MukVtkAxesActor::GetYAxisTipProperty()
{
  return mpYAxisTip->GetProperty();
}

//----------------------------------------------------------------------------
vtkProperty* MukVtkAxesActor::GetZAxisTipProperty()
{
  return mpZAxisTip->GetProperty();
}

//----------------------------------------------------------------------------
vtkProperty* MukVtkAxesActor::GetXAxisShaftProperty()
{
  return mpXAxisShaft->GetProperty();
}

//----------------------------------------------------------------------------
vtkProperty* MukVtkAxesActor::GetYAxisShaftProperty()
{
  return mpYAxisShaft->GetProperty();
}

//----------------------------------------------------------------------------
vtkProperty* MukVtkAxesActor::GetZAxisShaftProperty()
{
  return mpZAxisShaft->GetProperty();
}

//----------------------------------------------------------------------------
void MukVtkAxesActor::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
    
  os << indent << "ShaftType: " << mShaftType << endl;
  os << indent << "TipType: " << mTipType << endl;
  os << indent << "SphereRadius: " << mSphereRadius << endl;
  os << indent << "SphereResolution: " << mSphereResolution << endl;
  os << indent << "CylinderRadius: " << mCylinderRadius << endl;
  os << indent << "CylinderResolution: " << mCylinderResolution << endl;
  os << indent << "ConeRadius: " << mConeRadius << endl;
  os << indent << "ConeResolution: " << mConeResolution << endl;

  os << indent << "NormalizedShaftLength: "
    << mNormalizedShaftLength[0] << ","
    << mNormalizedShaftLength[1] << ","
    << mNormalizedShaftLength[2] << endl;

  os << indent << "NormalizedTipLength: "
    << mNormalizedTipLength[0] << ","
    << mNormalizedTipLength[1] << ","
    << mNormalizedTipLength[2] << endl;

  os << indent << "TotalLength: "
    << mTotalLength[0] << ","
    << mTotalLength[1] << ","
    << mTotalLength[2] << endl;

  os << indent << "NormalizedLabelPosition: "
    << mNormalizedLabelPosition[0] << ","
    << mNormalizedLabelPosition[1] << ","
    << mNormalizedLabelPosition[2] << endl;
}
