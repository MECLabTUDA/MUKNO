#include "private/muk.pch"
#include "SphereRegionRepresentation.h"

#include "MukCommon/geometry.h"
#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include "MukVisualization/muk_colors.h"

#include <vtkAssemblyPath.h>
#include <vtkCamera.h>
#include <vtkInteractorObserver.h>
#include <vtkProperty.h>
#include <vtkPropPicker.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>

#include <Eigen/Dense>

#include "MukCommon/gris_math.h"

namespace
{
  using namespace gris;
  using namespace gris::muk;

  void computeSphereAngleAxis(const Vec3d& v, Vec3d& axis, double& angle);
  void computeArrowAngleAxis(const Vec3d& v, Vec3d& axis, double& angle);
  void rotateMatrix(vtkTransform& trafo, const Vec3d& from_, const Vec3d& to_);
}

namespace gris
{
  namespace muk
  {
    vtkStandardNewMacro(SphereRegionRepresentation);

    /**
    */
    SphereRegionRepresentation::SphereContainer::SphereContainer()
    {
      sphere = make_vtk<vtkSphereSource>();
      matrix = make_vtk<vtkTransform>();
      trafo  = make_vtk<vtkTransformPolyDataFilter>();
      mapper = make_vtk<vtkPolyDataMapper>();
      actor  = make_vtk<vtkActor>();
      sphere->SetEndPhi(45);
      sphere->SetPhiResolution(30);
      sphere->SetThetaResolution(30);
      matrix->PostMultiply();
      matrix->Identity();
      trafo->SetTransform(matrix);
      trafo->SetInputConnection(sphere->GetOutputPort());
      mapper->SetInputConnection(trafo->GetOutputPort());
      actor->SetMapper(mapper);
      actor->GetProperty()->SetOpacity(0.25);
    }

    /**
    */
    SphereRegionRepresentation::ArrowContainer::ArrowContainer()
      : direction(0,0,1)
    {
      //arrow = Arrow::create(Vec3d(0,0,0), direction).getData();
      arrow  = make_vtk<vtkArrowSource>();
      matrix = make_vtk<vtkTransform>();
      trafo  = make_vtk<vtkTransformPolyDataFilter>();
      mapper = make_vtk<vtkPolyDataMapper>();
      actor  = make_vtk<vtkActor>();
      arrow->SetTipResolution(15);
      arrow->SetShaftResolution(15);
      matrix->PostMultiply();
      matrix->Identity();
      trafo->SetTransform(matrix);
      trafo->SetInputConnection(arrow->GetOutputPort());
      mapper->SetInputConnection(trafo->GetOutputPort());
      actor->SetMapper(mapper);
      actor->GetProperty()->SetColor(Colors::Orange);
    }

    /** \brief Default Initialisation of the Representation
    */
    SphereRegionRepresentation::SphereRegionRepresentation()
      : mBoundingBox(make_vtk<vtkBox>())
      , mPicker     (make_vtk<vtkCellPicker>())
      , mTransform  (make_vtk<vtkTransform>())
    {
      HandleSize = 5.0; /// default of implicitPlaneWidget
      mBumpDistance = 0.01;
      
      mTransform->PostMultiply();
      mTransform->Identity();

      //Manage the picking stuff
      mPicker->SetTolerance(0.005);
      mPicker->AddPickList(mSphere.actor);
      mPicker->AddPickList(mArrow.actor);
      mPicker->PickFromListOn();

      CreateDefaultProperties();

      InteractionState = enOutside;
    }

    /** \brief Default destruction of the Representation
    */
    SphereRegionRepresentation::~SphereRegionRepresentation() 
    {
    };

    MukState SphereRegionRepresentation::getState() const
    {
      return MukState(mSphere.origin, mArrow.direction);
    }

    /**
    */
    void SphereRegionRepresentation::SetInteractionState(int state)
    {
      InteractionState = state;;
      Highlight(state);
    }

    /**
    */
    int SphereRegionRepresentation::ComputeInteractionState(int X, int Y, int vtkNotUsed(modify))
    {
      // See if anything has been selected
      vtkAssemblyPath* path = GetAssemblyPath(X, Y, 0.0, mPicker);

      if ( path == nullptr ) // Not picking this widget
      {
        InteractionState = enOutside;
        return InteractionState;
      }

      // Something picked, continue
      ValidPick = 1;

      // Depending on the interaction state (set by the widget) we modify
      // this state based on what is picked.
      vtkProp *prop = path->GetFirstNode()->GetViewProp();
      if (prop == mSphere.actor.Get())
      {
        InteractionState = enMovingSphere;
      }
      else
      {
        double xyz[3];
        mPicker->GetPickPosition(xyz);
        // picked upper half of arrow?
        if ((mSphere.origin-Vec3d(xyz)).norm() > mSphere.sphere->GetRadius())
        {
          InteractionState = enRotating;
        }
        else
        {
          InteractionState = enMovingSphere;
        }
      }
      return InteractionState;
    }

    /**
    */
    void SphereRegionRepresentation::Highlight(int state)
    {
      switch (state)
      {
        case enMovingSphere:
        case enRotating:
          HighlightArrow(1);
          break;
        default:
          HighlightArrow(0);
      }
    }

    /**
    */
    void SphereRegionRepresentation::HighlightArrow(bool on)
    {
      if ( on )
      {
        mSphere.actor->SetProperty(mSphere.propActive);
        mArrow.actor->SetProperty(mArrow.propActive);
      }
      else
      {
        mSphere.actor->SetProperty(mSphere.propInactive);
        mArrow.actor->SetProperty(mArrow.propInactive);
      }
    }

    /**
    */
    void SphereRegionRepresentation::SetScale(double val)
    {
      mScale = val;
      mSphere.actor->SetScale(val);
      mArrow.actor->SetScale(val);
    }

    /**
    */
    double SphereRegionRepresentation::GetScale() const
    {
      return mScale;
    }

    /**
    */
    void SphereRegionRepresentation::setSpherePhi(double d)
    {
      mSphere.sphere->SetEndPhi(d);
      BuildRepresentation();
    }

    /**
    */
    double SphereRegionRepresentation::getSpherePhi() const
    {
      return mSphere.sphere->GetEndPhi();
    }

    /**
    */
    void SphereRegionRepresentation::setSphereRadius(double d)
    {
      mSphere.sphere->SetRadius(d);
      BuildRepresentation();
    }

    /**
    */
    double SphereRegionRepresentation::getSphereRadius() const
    {
      return mSphere.sphere->GetRadius();
    }
    
    /**
    */
    void SphereRegionRepresentation::PlaceWidget(const MukState& state)
    {
      mSphere.origin = state.coords;
      mArrow.direction = state.tangent;
      BuildRepresentation();
    }

    /**
    */
    void SphereRegionRepresentation::PlaceWidget(double* b)
    {
      mSphere.origin = 0.5*(Vec3d(b[0], b[2], b[4]) + Vec3d(b[1], b[3], b[5]));
      // whatever...
      double bounds[6], center[3];
      AdjustBounds(b, bounds, center);

      for (int i(0); i<6; ++i)
      {
        InitialBounds[i] = bounds[i];
      }
      InitialLength = sqrt((bounds[1]-bounds[0])*(bounds[1]-bounds[0]) +
        (bounds[3]-bounds[2])*(bounds[3]-bounds[2]) +
        (bounds[5]-bounds[4])*(bounds[5]-bounds[4]));

      ValidPick = 1; //since we have set up widget
      SizeHandles();
    }

    /**
    */
    void SphereRegionRepresentation::BuildRepresentation()
    {
      if ( ! Renderer )
      {
        return;
      }

      vtkInformation* info = GetPropertyKeys();
      mSphere.actor->SetPropertyKeys(info);
      mArrow.actor->SetPropertyKeys(info);

      if (GetMTime() > BuildTime)
      {
        // build the sphere transformation
        {
          auto& M = mSphere.matrix;
          M->Identity();
          rotateMatrix(*M, Vec3d(0,0,1), mArrow.direction);
          M->Translate(mSphere.origin.data());
          mSphere.trafo->SetTransform(M);
        }
        // build the arrow transformation
        {
          auto& M = mArrow.matrix;
          M->Identity();
          const double scale = 2*mSphere.sphere->GetRadius();
          M->Scale(scale, scale, scale);
          rotateMatrix(*M, Vec3d(1, 0, 0), mArrow.direction);
          M->Translate(mSphere.origin.data());
          mArrow.trafo->SetTransform(M);
        }
        SizeHandles();
        BuildTime.Modified();
      }
    }

    /**
    */
    void SphereRegionRepresentation::StartWidgetInteraction(double e[2])
    {
      StartEventPosition[0] = e[0];
      StartEventPosition[1] = e[1];
      StartEventPosition[2] = 0.0;

      mLastEventPosition[0] = e[0];
      mLastEventPosition[1] = e[1];
      mLastEventPosition[2] = 0.0;
    }

    /**
    */
    void SphereRegionRepresentation::WidgetInteraction(double e[2])
    {
      vtkCamera *camera = Renderer->GetActiveCamera();
      if ( !camera )
      {
        return;
      }
      
      // Do different things depending on state
      // Calculations everybody does
      double focalPoint[4], pickPoint[4], prevPickPoint[4];
      double z, vpn[3];

      // Compute the two points defining the motion vector
      double pos[3];
      mPicker->GetPickPosition(pos);
      vtkInteractorObserver::ComputeWorldToDisplay(Renderer, pos[0], pos[1], pos[2],focalPoint);
      z = focalPoint[2];
      vtkInteractorObserver::ComputeDisplayToWorld(Renderer, mLastEventPosition[0], mLastEventPosition[1], z, prevPickPoint);
      vtkInteractorObserver::ComputeDisplayToWorld(Renderer, e[0], e[1], z, pickPoint);
      // Process the motion
      switch (InteractionState)
      {
        case enMovingSphere:
          TranslateSphere(prevPickPoint, pickPoint);
          break;
        case enRotating:
          camera->GetViewPlaneNormal(vpn);
          RotateCone(e[0], e[1], prevPickPoint, pickPoint, vpn);
          break;
        case enPlaceSphere:
          placeSphere(e);
          break;
      }

      mLastEventPosition[0] = e[0];
      mLastEventPosition[1] = e[1];
      mLastEventPosition[2] = 0.0;
    }

    /**
    */
    void SphereRegionRepresentation::EndWidgetInteraction(double newEventPos[2])
    {
      
    }

    /**
    */
    double* SphereRegionRepresentation::GetBounds()
    {
      BuildRepresentation();
      mBoundingBox->SetBounds(mSphere.actor->GetBounds());
      mBoundingBox->AddBounds(mArrow.actor->GetBounds());
      return mBoundingBox->GetBounds();
    }

    /**
    */
   /* void SphereRegionRepresentation::GetActors(vtkPropCollection *pc)
    {
      mSphere.actor->GetActors(pc);
      mArrow.actor->GetActors(pc);
    }
*/
    /**
    */
    void SphereRegionRepresentation::ReleaseGraphicsResources(vtkWindow* w)
    {
      mSphere.actor->ReleaseGraphicsResources(w);
      mArrow.actor->ReleaseGraphicsResources(w);
    }

    /**
    */
    int SphereRegionRepresentation::RenderOpaqueGeometry(vtkViewport* v)
    {
      int count=0;
      count += mSphere.actor->RenderOpaqueGeometry(v);
      count += mArrow.actor->RenderOpaqueGeometry(v);
      return count;
    }

    /**
    */
    int SphereRegionRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport* v)
    {
      int count=0;
      count += mSphere.actor->RenderTranslucentPolygonalGeometry(v);
      count += mArrow.actor->RenderTranslucentPolygonalGeometry(v);
      return count;
    }

    /**
    */
    int SphereRegionRepresentation::HasTranslucentPolygonalGeometry()
    {
      int result=0;
      result |= mSphere.actor->HasTranslucentPolygonalGeometry();
      result |= mArrow.actor->HasTranslucentPolygonalGeometry();
      return result;
    }

    /**
    */
    void SphereRegionRepresentation::placeSphere(double* e)
    {
      // Pick from this location.
      auto picker = make_vtk<vtkPropPicker>();
      picker->Pick(e[0], e[1], 0, Renderer);
      if (picker->GetActor() == nullptr)
        return;
      double* pos = picker->GetPickPosition();
      // place the origin
      for (size_t i(0); i<3; ++i)
        mSphere.origin[i] = pos[i];
      Modified();
      BuildRepresentation();
    }

    /**
    */
    void SphereRegionRepresentation::TranslateSphere(double *p1, double *p2)
    {
      // move the origin according to the motion vector
      for (size_t i(0); i<3; ++i)
        mSphere.origin[i] += p2[i] - p1[i];
      Modified();
      BuildRepresentation();
    }
    
    /**
    */
    void SphereRegionRepresentation::RotateCone(double X, double Y, double * p1, double * p2, double * coneDirection)
    {
      double v[3];    //vector of motion
      double axis[3]; //axis of rotation
      double theta;   //rotation angle

      // mouse motion vector in world space
      v[0] = p2[0] - p1[0];
      v[1] = p2[1] - p1[1];
      v[2] = p2[2] - p1[2];

      double *origin = mSphere.origin.data();
      const Vec3d& oldDirection = mArrow.direction; // vtk's not-const-correctness is anoying

      // Create axis of rotation and angle of rotation
      vtkMath::Cross(coneDirection, v, axis);
      if (vtkMath::Normalize(axis) == 0.0)
      {
        return;
      }
      int *size = Renderer->GetSize();
      double l2 = (X - mLastEventPosition[0])*(X - mLastEventPosition[0]) + (Y - mLastEventPosition[1])*(Y - mLastEventPosition[1]);
      theta = 360.0 * sqrt(l2 / (size[0] * size[0] + size[1] * size[1]));

      // Manipulate the transform to reflect the rotation
      mTransform->Identity();
      mTransform->PostMultiply();
      //mTransform->Translate(origin[0], origin[1], origin[2]);
      mTransform->RotateWXYZ(theta, axis);
      //mTransform->Translate(-origin[0], -origin[1], -origin[2]);
      
      //Set the new Direction
      double nNew[3];
      mTransform->TransformNormal(oldDirection.data(), nNew);
      mArrow.direction = Vec3d(nNew).normalized();
      Modified();
      BuildRepresentation();
    }

    /**
    */
    void SphereRegionRepresentation::SizeHandles()
    {
    }

    /**
    */
    void SphereRegionRepresentation::CreateDefaultProperties()
    {
      mSphere.propActive = make_vtk<vtkProperty>();
      mSphere.propActive->SetColor(Colors::Green);
      mSphere.propActive->SetOpacity(0.5);

      mSphere.propInactive= make_vtk<vtkProperty>();
      mSphere.propInactive->SetColor(Colors::Orange);
      mSphere.propInactive->SetOpacity(0.5);

      mArrow.propActive = make_vtk<vtkProperty>();
      mArrow.propActive->SetColor(Colors::Green);

      mArrow.propInactive = make_vtk<vtkProperty>();
      mArrow.propInactive->SetColor(Colors::Orange);
    }
    
    /**
    */
    void SphereRegionRepresentation::setHandles(bool on)
    {
      if (on)
      {
        mArrow.actor->SetProperty(mArrow.propActive);
        mSphere.actor->SetProperty(mSphere.propActive);
      }
      else
      {
        mArrow.actor->SetProperty(mArrow.propInactive);
        mSphere.actor->SetProperty(mSphere.propInactive);
      }
    }

    /**
    */
    void SphereRegionRepresentation::setArrowDirection(const Vec3d& v)
    {
      mArrow.direction= v;
      BuildRepresentation();
    }

    /**
    */
    void SphereRegionRepresentation::setSphereOrigin(const Vec3d& p)
    {
      mSphere.origin = p;
      BuildRepresentation();
    }
  }
}

namespace
{
  /** \brief computes the angle axis representation of a rotation from the arrow orientation onto the vector v  
  */
  void computeArrowAngleAxis(const Vec3d& v, Vec3d& axis, double& angle)
  {
    const auto n = Vec3d(1,0,0); // arrow source's fixed direction
    axis  = v.cross(n).normalized();
    angle = -std::acos(std::abs(v.dot(n))) * 180 / gris::M_Pi;
    angle = -wideAngle(v,n) * 180 / gris::M_Pi;
  }

  /** \brief computes the angle axis representation of a rotation from the sphere orientation onto the vector v
  */
  void computeSphereAngleAxis(const Vec3d& v, Vec3d& axis, double& angle)
  {
    const auto n = Vec3d(0,0,1); // sphere source's fixed direction
    axis  = v.cross(n).normalized();
    angle = -std::acos(std::abs(v.dot(n))) * 180 / gris::M_Pi;
    angle = -wideAngle(v,n) * 180 / gris::M_Pi;
  }

  /**
  */
  void rotateMatrix(vtkTransform& trafo, const Vec3d& from_, const Vec3d& to_)
  {
    using namespace Eigen;
    Vector3d from(from_.normalized().data());
    Vector3d to(to_.normalized().data());
    Vector3d v = from.cross(to);
    const double c = from.dot(to);

    Eigen::Matrix3d R;
    R(0,0) =      0;   R(0,1) = -v.z();   R(0,2) =  v.y();
    R(1,0) =  v.z();   R(1,1) =      0;   R(1,2) = -v.x();
    R(2,0) = -v.y();   R(2,1) =  v.x();   R(2,2) =      0;
    R = Eigen::Matrix3d::Identity() + R + 1 / (1+c) * R*R;
    Vector3d result = R*from;      
    Vec3d res(result.x(), result.y(), result.z());
    
    AngleAxisd rep;
    rep.fromRotationMatrix(R);
    trafo.RotateWXYZ(rep.angle()*180/gris::M_Pi, rep.axis().data());
  }
}