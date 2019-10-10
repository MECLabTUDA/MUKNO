#include "private/muk.pch"
#include "MultiPortSphereRegionRepresentation.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/gris_math.h"
#include "MukCommon/vtk_tools.h"

#include <vtkAssemblyPaths.h>
#include <vtkCamera.h>
#include <vtkInteractorObserver.h>
#include <vtkBox.h>
#include <vtkTransform.h>
#include <vtkObject.h>
#include <vtkTubeFilter.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkMatrix4x4.h>
#include <vtkMatrix3x3.h>

namespace
{
  using namespace gris;
  using namespace gris::muk;
  const double mArrowScaling = 1.2;

  void setMatrix(vtkTransform* M, const Vec3d& dir, double T[3]);
}

namespace gris
{
  namespace muk
  {
    vtkStandardNewMacro(MultiPortSphereRegionRepresentation);

    /**
    */
    MultiPortSphereRegionRepresentation::Arrow::Arrow()
    {
      mLine = make_vtk<vtkLineSource>();
      mTube = make_vtk<vtkTubeFilter>();
      mTubeMapper = make_vtk<vtkPolyDataMapper>();
      mTubeActor  = make_vtk<vtkActor>();
      mLine->SetResolution(0.1);

      mTube->SetCapping(true);
      mTube->SetRadius(1.0);
      mTube->SetNumberOfSides(50);
      mTube->SetInputData(mLine->GetOutput());

      mTubeMapper->SetInputConnection(mTube->GetOutputPort());
      mTubeActor ->SetMapper(mTubeMapper);

      mLineMapper = make_vtk<vtkPolyDataMapper>();
      mLineActor  = make_vtk<vtkActor>();
      mLineMapper->SetInputConnection(mLine->GetOutputPort());
      mLineActor ->SetMapper(mLineMapper);
      
      mThresh        = make_vtk<vtkSphereSource>();
      // default parameters
      mThresh->SetEndPhi(5); 
      mThresh->SetRadius(1.0);
      mThresh->SetPhiResolution(30);
      mThresh->SetThetaResolution(30);
      mThreshTrafo   = make_vtk<vtkTransformPolyDataFilter>();
      {
        auto T = make_vtk<vtkTransform>();
        auto M = make_vtk<vtkMatrix4x4>();
        M->Identity();
        T->SetMatrix(M);
        mThreshTrafo->SetTransform(T);
      }
      mThreshMapper  = make_vtk<vtkPolyDataMapper>();
      mThreshActor   = make_vtk<vtkActor>();
      mThreshTrafo ->SetInputConnection(mThresh->GetOutputPort());
      mThreshMapper->SetInputConnection(mThreshTrafo->GetOutputPort());
      mThreshActor ->SetMapper(mThreshMapper);
      mThreshActor->GetProperty()->SetColor(0, 1, 1);
      mThreshActor->GetProperty()->SetOpacity(0.5);
    }

    /** \brief Default Initialisation of the Representation
    */
    MultiPortSphereRegionRepresentation::MultiPortSphereRegionRepresentation()
      : mPathRadius(1.0)
      , mPortActivity(1,1,1)
    {
      Sphere->SetPhiResolution(30);
      Sphere->SetThetaResolution(30);
      // Set default direction of the Arrows, so there is an initialized value
      mArrows[0].mDirection = Vec3d(1, 2, 2).normalized();
      mArrows[1].mDirection = Vec3d(2, 2, 1).normalized();
      mArrows[2].mDirection = Vec3d(2, 1, 2).normalized();

      for (int i(0); i < 3; ++i)
      {
        mArrows[i].tube()->SetRadius(mPathRadius);
        this->Picker->AddPickList(mArrows[i].tubeActor()); // Manage the picking stuff of the Arrows
      }
      
      this->CreateDefaultProperties();
      // Pass the initial properties to the actors
      for (int i(0); i < 3; ++i)
      {
        mArrows[i].lineActor()->SetProperty(this->mpArrowProperty);
        mArrows[i].tubeActor()->SetProperty(this->mpArrowProperty);
      }
    }

    /** \brief Default destruction of the Representation
    */
    MultiPortSphereRegionRepresentation::~MultiPortSphereRegionRepresentation() 
    {
    };

    /**
    */
    void MultiPortSphereRegionRepresentation::setPathRadius(double d)
    {
      mPathRadius = d;
      for (int i(0); i<3; ++i)
      {
        mArrows[i].tube()->SetRadius(mPathRadius);
      }
    }

    /**
    */
    void MultiPortSphereRegionRepresentation::setSphereRadius(double d)
    {
      Sphere->SetRadius(d);
      for (int i(0); i<3; ++i)
        mArrows[i].mThresh->SetRadius(d);
    }

    /**
    */
    void MultiPortSphereRegionRepresentation::setAngleThreshold(double alpha)
    {
      mAngleThreshold = alpha;
      for (int i(0); i<3; ++i)
        mArrows[i].mThresh->SetEndPhi(alpha*180/gris::M_Pi);
    }

    /**
    */
    MukState MultiPortSphereRegionRepresentation::getState(size_t i)
    {
      MukState res;
      res.coords  = Vec3d(Sphere->GetCenter());
      res.tangent = mArrows[i].mDirection;
      return res;
    }

    /**
    */
    double MultiPortSphereRegionRepresentation::getSphereRadius() const
    {
      return Sphere->GetRadius();
    }

    /**
    */
    void MultiPortSphereRegionRepresentation::centralize(const Vec3d& p)
    {
      auto* b = GetBounds();
      const auto extent = 0.5*Vec3d(b[1]-b[0], b[3]-b[2], b[5]-b[4]);
      double bnew[6];
      for (int i(0); i<3; ++i)
      {
        bnew[2*i]   = p[i] - extent[i];
        bnew[2*i+1] = p[i] + extent[i];
      }
      PlaceWidget(bnew);
      Plane->SetOrigin(const_cast<Vec3d&>(p).data());
    }

    /** *\brief checks wich Interactionstate is activated
    *
    * @param X x-coordinate of the mouse
    * @param Y y-coordinate of the mouse
    * 
    */
    int MultiPortSphereRegionRepresentation::ComputeInteractionState(int X, int Y, int vtkNotUsed(modify))
    {
      // See if anything has been selected
      vtkAssemblyPath* path = this->GetAssemblyPath(X, Y, 0., this->Picker);

      if (path == nullptr) // Not picking this widget
      {
        this->SetRepresentationState(vtkImplicitPlaneRepresentation::Outside);
        this->InteractionState = vtkImplicitPlaneRepresentation::Outside;
        return this->InteractionState;
      }

      // Something picked, continue
      this->ValidPick = 1;

      // Depending on the interaction state (set by the widget) we modify
      // this state based on what is picked.
      if (this->InteractionState == vtkImplicitPlaneRepresentation::Moving)
      {
        vtkProp *prop = path->GetFirstNode()->GetViewProp();
        if ((prop == this->ConeActor || prop == this->LineActor ||
          prop == this->ConeActor2 || prop == this->LineActor2))
        {
          this->InteractionState = vtkImplicitPlaneRepresentation::Rotating;
          this->SetRepresentationState(vtkImplicitPlaneRepresentation::Rotating);
        }
        else if (prop == mArrows[0].tubeActor())
        {
          this->InteractionState = MultiPortSphereRegionRepresentation::RotatingArrow1;
          this->SetRepresentationState(MultiPortSphereRegionRepresentation::RotatingArrow1);
        }
        else if (prop == mArrows[1].tubeActor())
        {
          this->InteractionState = MultiPortSphereRegionRepresentation::RotatingArrow2;
          this->SetRepresentationState(MultiPortSphereRegionRepresentation::RotatingArrow2);
        }
        else if (prop == mArrows[2].tubeActor())
        {
          this->InteractionState = MultiPortSphereRegionRepresentation::RotatingArrow3;
          this->SetRepresentationState(MultiPortSphereRegionRepresentation::RotatingArrow3);
        }
        else if (prop == this->CutActor)
        {
          if (this->LockNormalToCamera)
          { // Allow camera to work
            this->InteractionState = vtkImplicitPlaneRepresentation::Outside;
            this->SetRepresentationState(vtkImplicitPlaneRepresentation::Outside);
          }
          else
          {
            this->InteractionState = vtkImplicitPlaneRepresentation::Pushing;
            this->SetRepresentationState(vtkImplicitPlaneRepresentation::Pushing);
          }
        }
        else if (prop == this->SphereActor)
        {
          this->InteractionState = vtkImplicitPlaneRepresentation::MovingOrigin;
          this->SetRepresentationState(vtkImplicitPlaneRepresentation::MovingOrigin);
        }
        else
        {
          if (this->OutlineTranslation)
          {
            this->InteractionState = vtkImplicitPlaneRepresentation::MovingOutline;
            this->SetRepresentationState(vtkImplicitPlaneRepresentation::MovingOutline);
          }
          else
          {
            this->InteractionState = vtkImplicitPlaneRepresentation::Outside;
            this->SetRepresentationState(vtkImplicitPlaneRepresentation::Outside);
          }
        }
      }

      // We may add a condition to allow the camera to work IO scaling
      else if (this->InteractionState != vtkImplicitPlaneRepresentation::Scaling)
      {
        this->InteractionState = vtkImplicitPlaneRepresentation::Outside;
      }

      return this->InteractionState;
    }

    /** \brief Builds the the representation based on the normal and origin of the plane

      typical reinitialization procedure after more or less any changing event
    */
    void MultiPortSphereRegionRepresentation::BuildRepresentation()
    {
      if (!this->Renderer || !this->Renderer->GetRenderWindow())
      {
        return;
      }
      
      vtkInformation *info = this->GetPropertyKeys();
      this->OutlineActor->SetPropertyKeys(info);
      this->CutActor->SetPropertyKeys(info);
      this->EdgesActor->SetPropertyKeys(info);
      this->ConeActor->SetPropertyKeys(info);
      this->LineActor->SetPropertyKeys(info);
      this->ConeActor2->SetPropertyKeys(info);
      this->LineActor2->SetPropertyKeys(info);
      this->SphereActor->SetPropertyKeys(info);
      for (int i(0); i<3; ++i)
      {
        mArrows[i].tubeActor()->SetPropertyKeys(info);
        mArrows[i].lineActor()->SetPropertyKeys(info);
        //mArrows[i].mThreshActor->SetPropertyKeys(info);
      }

      if (this->GetMTime() > this->BuildTime ||
        this->Plane->GetMTime() > this->BuildTime ||
        this->Renderer->GetRenderWindow()->GetMTime() > this->BuildTime)
      {
        // ==========
        // first some base class vtk procedure
        // ==========
        // Origin of the plane
        double *origin = this->Plane->GetOrigin();
        // Normal of the plane
        double *normal = this->Plane->GetNormal();

        // Bounds of the Widget
        double bounds[6];
        std::copy(this->WidgetBounds, this->WidgetBounds + 6, bounds);

        // Center of the Cones | vector of the Lines of the Arrows
        double p2[3];

        // Checks if the Origin traverse the Bounds
        if (!this->OutsideBounds)
        {
          // restrict the origin inside InitialBounds
          double *ibounds = this->InitialBounds;
          for (int i = 0; i < 3; i++)
          {
            if (origin[i] < ibounds[2 * i])
            {
              origin[i] = ibounds[2 * i];
            }
            else if (origin[i] > ibounds[2 * i + 1])
            {
              origin[i] = ibounds[2 * i + 1];
            }
          }
        }

        if (this->ConstrainToWidgetBounds)
        {
          if (!this->OutsideBounds)
          {
            // origin cannot move outside InitialBounds. Therefore, restrict
            // movement of the Box.
            double v[3] = { 0.0, 0.0, 0.0 };
            for (int i = 0; i < 3; ++i)
            {
              if (origin[i] <= bounds[2 * i])
              {
                v[i] = origin[i] - bounds[2 * i] - FLT_EPSILON;
              }
              else if (origin[i] >= bounds[2 * i + 1])
              {
                v[i] = origin[i] - bounds[2 * i + 1] + FLT_EPSILON;
              }
              bounds[2 * i] += v[i];
              bounds[2 * i + 1] += v[i];
            }
          }

          // restrict origin inside bounds
          for (int i = 0; i < 3; ++i)
          {
            if (origin[i] <= bounds[2 * i])
            {
              origin[i] = bounds[2 * i] + FLT_EPSILON;
            }
            if (origin[i] >= bounds[2 * i + 1])
            {
              origin[i] = bounds[2 * i + 1] - FLT_EPSILON;
            }
          }
        }
        else // plane can move freely, adjust the bounds to change with it
        {
          double offset = this->Box->GetLength() * 0.02;
          for (int i = 0; i < 3; ++i)
          {
            bounds[2 * i] = vtkMath::Min(origin[i] - offset, this->WidgetBounds[2 * i]);
            bounds[2 * i + 1] = vtkMath::Max(origin[i] + offset, this->WidgetBounds[2 * i + 1]);
          }
        }

        this->Box->SetOrigin(bounds[0], bounds[2], bounds[4]);
        this->Box->SetSpacing((bounds[1] - bounds[0]), (bounds[3] - bounds[2]),
          (bounds[5] - bounds[4]));
        this->Outline->Update();

        // Setup the length of the lines
        double d = Sphere->GetRadius()*2.0;

        p2[0] = origin[0] + d * normal[0];
        p2[1] = origin[1] + d * normal[1];
        p2[2] = origin[2] + d * normal[2];

        this->LineSource->SetPoint1(origin);
        this->LineSource->SetPoint2(p2);
        this->ConeSource->SetCenter(p2);
        this->ConeSource->SetDirection(normal);

        p2[0] = origin[0] - d * normal[0];
        p2[1] = origin[1] - d * normal[1];
        p2[2] = origin[2] - d * normal[2];

        this->LineSource2->SetPoint1(origin[0], origin[1], origin[2]);
        this->LineSource2->SetPoint2(p2);
        this->ConeSource2->SetCenter(p2);
        this->ConeSource2->SetDirection(normal[0], normal[1], normal[2]);

        // ==========
        // start of additional procedure
        // ==========
        double arrowLength = mArrowScaling*Sphere->GetRadius();
        // Calculates the rotation for Arrows
        // rotated = new direction of the rotated Arrow
        for (int i(0); i < 3; ++i)
        {
          Vec3d p1, p2;
          for (int j(0); j < 3; ++j)
          {
            p1[j] = origin[j];
            p2[j] = origin[j] + arrowLength * mArrows[i].mDirection[j];
          }
          mArrows[i].line()->SetPoint1(p1.data());
          mArrows[i].line()->SetPoint2(p2.data());
          auto T = make_vtk<vtkTransform>();
          setMatrix(T, mArrows[i].mDirection, origin);
          mArrows[i].mThreshTrafo->SetTransform(T);
        }
        
        // Set up the Origin
        this->Sphere->SetCenter(origin[0], origin[1], origin[2]);

        // Control the look of the edges
        if (this->Tubing)
        {
          this->EdgesMapper->SetInputConnection(
            this->EdgesTuber->GetOutputPort());
        }
        else
        {
          this->EdgesMapper->SetInputConnection(
            this->Edges->GetOutputPort());
        }
        this->SizeHandles();
        this->BuildTime.Modified();
      }
    }

    /**
    * \brief Manages the the size of the Cones if Zoomed
    */
    void MultiPortSphereRegionRepresentation::SizeHandles()
    {
      // radius scales with the zoom to pick actors better (not used for cones but it may be changed here)
      double radius = this->vtkWidgetRepresentation::SizeHandlesInPixels(1.5, this->Sphere->GetCenter());
      // defaultRadius for the Arrows' Cones (change to scale)
      const auto r = Sphere->GetRadius();
      double defaultRadius = r * 0.2;
      this->ConeSource->SetHeight(defaultRadius);
      this->ConeSource->SetRadius(defaultRadius);
      this->ConeSource2->SetHeight(defaultRadius);
      this->ConeSource2->SetRadius(defaultRadius);
      this->Sphere->SetRadius(r);
      this->EdgesTuber->SetRadius(0.25*radius);
    }

    /**
    * \brief rotates the Plane to the Arrowmotion
    *
    * @param X x - coordinate of the mouse
    * @param Y y - coordinate of the mouse
    * @param p1 Old mouse position
    * @param p2 New mouse position
    * @param vpn ViewPlaneNormal (old normale of the plane)
    *
    */
    void MultiPortSphereRegionRepresentation::Rotate(double X, double Y,
      double *p1, double *p2, double *vpn)
    {
      double v[3]; //vector of motion
      double axis[3]; //axis of rotation
      double theta; //rotation angle

      // mouse motion vector in world space
      v[0] = p2[0] - p1[0];
      v[1] = p2[1] - p1[1];
      v[2] = p2[2] - p1[2];

      double *origin = this->Plane->GetOrigin();
      double *normal = this->Plane->GetNormal();

      // Create axis of rotation and angle of rotation
      vtkMath::Cross(vpn, v, axis);
      if (vtkMath::Normalize(axis) == 0.0)
      {
        return;
      }
      int *size = this->Renderer->GetSize();
      double l2 = (X - this->LastEventPosition[0])*(X - this->LastEventPosition[0]) + (Y - this->LastEventPosition[1])*(Y - this->LastEventPosition[1]);
      theta = 360.0 * sqrt(l2 / (size[0] * size[0] + size[1] * size[1]));

      //Manipulate the transform to reflect the rotation
      this->Transform->Identity();
      this->Transform->Translate(origin[0], origin[1], origin[2]);
      this->Transform->RotateWXYZ(theta, axis);
      this->Transform->Translate(-origin[0], -origin[1], -origin[2]);

      //Set the new normal
      double nNew[3];
      this->Transform->TransformNormal(normal, nNew);
      this->SetNormal(nNew);
      for (int i(0); i < 3; ++i)
      {
        this->Transform->TransformNormal(mArrows[i].mDirection.data(), mArrows[i].mDirection.data());
      }
    }

    /** \brief Rotates a certain cone

      \param X mouse position on screen
      \param Y mouse position on screen
      \param p1 last mouse position (world)
      \param p2 new mouse position (world)
      \param coneDirection
    */
    void MultiPortSphereRegionRepresentation::RotateCone(double X, double Y, double * p1, double * p2, double * coneDirection)
    {
      double v[3]; //vector of motion
      double axis[3]; //axis of rotation
      double theta; //rotation angle

      // mouse motion vector in world space
      v[0] = p2[0] - p1[0];
      v[1] = p2[1] - p1[1];
      v[2] = p2[2] - p1[2];

      double *origin = this->Plane->GetOrigin();

      Vec3d conesDirection;
      if (this->InteractionState == MultiPortSphereRegionRepresentation::RotatingArrow1)
        conesDirection = mArrows[0].mDirection;
      else if (this->InteractionState == MultiPortSphereRegionRepresentation::RotatingArrow2)
        conesDirection = mArrows[1].mDirection;
      else if (this->InteractionState == MultiPortSphereRegionRepresentation::RotatingArrow3)
        conesDirection = mArrows[2].mDirection;

      // Create axis of rotation and angle of rotation
      vtkMath::Cross(coneDirection, v, axis);
      if (vtkMath::Normalize(axis) == 0.0)
      {
        return;
      }
      int *size = this->Renderer->GetSize();
      double l2 = (X - this->LastEventPosition[0])*(X - this->LastEventPosition[0]) + (Y - this->LastEventPosition[1])*(Y - this->LastEventPosition[1]);
      theta = 360.0 * sqrt(l2 / (size[0] * size[0] + size[1] * size[1]));

      //Manipulate the transform to reflect the rotation
      this->Transform->Identity();
      this->Transform->Translate(origin[0], origin[1], origin[2]);
      this->Transform->RotateWXYZ(theta, axis);
      this->Transform->Translate(-origin[0], -origin[1], -origin[2]);


      //Set the new Direction
      double nNew[3];
      this->Transform->TransformNormal(conesDirection.data(), nNew);

      if (this->InteractionState == MultiPortSphereRegionRepresentation::RotatingArrow1)
        mArrows[0].mDirection = Vec3d(nNew);
      else if (this->InteractionState == MultiPortSphereRegionRepresentation::RotatingArrow2)
        mArrows[1].mDirection = Vec3d(nNew);
      else if (this->InteractionState == MultiPortSphereRegionRepresentation::RotatingArrow3)
        mArrows[2].mDirection = Vec3d(nNew);
      this->Modified();
    }

    /** \brief Scales Origin and Box
    */
    void MultiPortSphereRegionRepresentation::Scale(double *p1, double *p2, double vtkNotUsed(X), double Y, double * vpn)
    {
      //Get the motion vector
      double v[3];
      v[0] = p2[0] - p1[0];
      v[1] = p2[1] - p1[1];
      v[2] = p2[2] - p1[2];

      double *o = this->Plane->GetOrigin();

      // Compute the scale factor
      double sf = vtkMath::Norm(v) / this->Outline->GetOutput()->GetLength();
      if (Y > this->LastEventPosition[1])
      {
        sf = 1.0 + sf;
      }
      else
      {
        sf = 1.0 - sf;
      }

      this->Transform->Identity();
      this->Transform->Translate(o[0], o[1], o[2]);
      this->Transform->Scale(sf, sf, sf);
      this->Transform->Translate(-o[0], -o[1], -o[2]);

      double *origin = this->Box->GetOrigin();
      double *spacing = this->Box->GetSpacing();
      double oNew[3], p[3], pNew[3];
      p[0] = origin[0] + spacing[0];
      p[1] = origin[1] + spacing[1];
      p[2] = origin[2] + spacing[2];

      this->Transform->TransformPoint(origin, oNew);
      this->Transform->TransformPoint(p, pNew);

      this->Box->SetOrigin(oNew);
      this->Box->SetSpacing((pNew[0] - oNew[0]),
        (pNew[1] - oNew[1]),
        (pNew[2] - oNew[2]));
      this->Box->GetBounds(this->WidgetBounds);
      this->setSphereRadius(this->Outline->GetOutput()->GetLength() * 0.1);
      this->SizeHandles();
      this->BuildRepresentation();
    }

    /** \brief Set the normal to the plane and calculates the rotation of the normal
      *
      * @param x, y, z New normalvector
    */
    void MultiPortSphereRegionRepresentation::SetNormal(double x, double y, double z)
    {
      double n[3], n2[3];
      n[0] = x;
      n[1] = y;
      n[2] = z;
      vtkMath::Normalize(n);

      this->Plane->GetNormal(n2);
      if (n[0] != n2[0] || n[1] != n2[1] || n[2] != n2[2])
      {
        //// Compute rotation matrix between two vectors
        //double vec[3];
        //vtkMath::Cross(n, n2, vec);
        //double costheta = vtkMath::Dot(n, n2);
        //double sintheta = vtkMath::Norm(vec);
        //double theta = atan2(sintheta, costheta);
        //if (sintheta != 0)
        //{
        //  vec[0] /= sintheta;
        //  vec[1] /= sintheta;
        //  vec[2] /= sintheta;
        //}
        //// convert to quaternion
        //costheta = cos(0.5*theta);
        //sintheta = sin(0.5*theta);
        //double quat[4];
        //quat[0] = costheta;
        //quat[1] = vec[0] * sintheta;
        //quat[2] = vec[1] * sintheta;
        //quat[3] = vec[2] * sintheta;
        this->Plane->SetNormal(n);
        this->Modified();
      }
    }

    /** \brief LockNormalToCamera will cause the normal to follow the camera's normal
      *
      * @param lock Wether Camera is locked or not
    */
    void MultiPortSphereRegionRepresentation::SetLockNormalToCamera(int lock)
    {
      vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting "
        << LockNormalToCamera << " to " << lock);

      if (lock == this->LockNormalToCamera)
      {
        return;
      }

      if (lock)
      {
        this->Picker->DeletePickList(this->LineActor);
        this->Picker->DeletePickList(this->ConeActor);
        this->Picker->DeletePickList(this->LineActor2);
        this->Picker->DeletePickList(this->ConeActor2);
        for (int i(0); i<3; ++i)
        {
          this->Picker->DeletePickList(mArrows[i].tubeActor());
        }
        this->Picker->DeletePickList(this->SphereActor);

        this->SetNormalToCamera();
      }
      else
      {
        this->Picker->AddPickList(this->LineActor);
        this->Picker->AddPickList(this->ConeActor);
        this->Picker->AddPickList(this->LineActor2);
        this->Picker->AddPickList(this->ConeActor2);
        for (int i(0); i<3; ++i)
        {
          this->Picker->AddPickList(mArrows[i].tubeActor());
        }
        this->Picker->AddPickList(this->SphereActor);
      }

      this->LockNormalToCamera = lock;
      this->Modified();
    }

    /**
    * \brief Highlights the Objects weather it's their Representation state
    * 
    * @param state The new Representationstate
    */
    void MultiPortSphereRegionRepresentation::SetRepresentationState(int state)
    {
      if (this->RepresentationState == state)
      {
        return;
      }

      // Clamp the state
      state = (state < vtkImplicitPlaneRepresentation::Outside ?
        vtkImplicitPlaneRepresentation::Outside :
        (state > vtkImplicitPlaneRepresentation::Scaling ?
          vtkImplicitPlaneRepresentation::Scaling : state));

      this->RepresentationState = state;
      this->Modified();

      // Highlights for Rotating
      if (state == vtkImplicitPlaneRepresentation::Rotating)
      {
        this->HighlightNormal(1);
        this->HighlightPlane(1);
      }
      // Highlights for Pushing
      else if (state == vtkImplicitPlaneRepresentation::Pushing)
      {
        this->HighlightNormal(1);
        this->HighlightPlane(1);
      }
      // Highlights for MoveOrigin
      else if (state == vtkImplicitPlaneRepresentation::MovingOrigin)
      {
        this->HighlightNormal(1);
      }
      // Highlights for MovingOutline
      else if (state == vtkImplicitPlaneRepresentation::MovingOutline)
      {
        this->HighlightOutline(1);
      }
      // Highlights for Scaling
      else if (state == vtkImplicitPlaneRepresentation::Scaling &&
        this->ScaleEnabled)
      {
        this->HighlightNormal(1);
        this->HighlightPlane(1);
        this->HighlightOutline(1);
      }
      // Highlights for rotating Arrows1
      else if (state == MultiPortSphereRegionRepresentation::RotatingArrow1)
      {
        this->HighlightNormal(1);
        this->HighlightPlane(1);
      }
      // Highlights for rotating Arrows
      else if (state == MultiPortSphereRegionRepresentation::RotatingArrow2)
      {
        this->HighlightNormal(1);
        this->HighlightPlane(1);
      }
      // Highlights for rotating Arrows
      else if (state == MultiPortSphereRegionRepresentation::RotatingArrow3)
      {
        this->HighlightNormal(1);
        this->HighlightPlane(1);
      }
      // Highlights for Default
      else
      {
        this->HighlightNormal(0);
        this->HighlightPlane(0);
        this->HighlightOutline(0);
      }
    }

    /**
    *  \brief Starts calculation for the Interactions
    *
    * @param e[2] Mouseposition on screen
    */
    void MultiPortSphereRegionRepresentation::WidgetInteraction(double e[2])
    {
      // Do different things depending on state
      double focalPoint[4], pickPoint[4], prevPickPoint[4];
      double z, vpn[3];

      vtkCamera *camera = this->Renderer->GetActiveCamera();
      if (!camera)
      {
        return;
      }

      // Compute the two points defining the motion vector
      double pos[3];
      this->Picker->GetPickPosition(pos);
      vtkInteractorObserver::ComputeWorldToDisplay(this->Renderer, pos[0], pos[1], pos[2],
        focalPoint);
      z = focalPoint[2];
      vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer, this->LastEventPosition[0],
        this->LastEventPosition[1], z, prevPickPoint);
      vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer, e[0], e[1], z, pickPoint);

      auto getDirection = [&](Arrow& arrow, double* output)
      {
        Vec3d p1, p2;
        arrow.line()->GetPoint1(p1.data());
        arrow.line()->GetPoint2(p2.data());
        auto dir = (p2 - p1).normalized();
        for (int i(0); i < 3; ++i)
          output[i] = dir[i];
      };

      // Process the motion
      if (this->InteractionState == vtkImplicitPlaneRepresentation::MovingOutline)
      {
        this->TranslateOutline(prevPickPoint, pickPoint);
      }
      else if (this->InteractionState == vtkImplicitPlaneRepresentation::MovingOrigin)
      {
        this->TranslateOrigin(prevPickPoint, pickPoint);
      }
      else if (this->InteractionState == vtkImplicitPlaneRepresentation::Pushing)
      {
        this->Push(prevPickPoint, pickPoint);
      }
      else if (this->InteractionState == MultiPortSphereRegionRepresentation::Scaling &&
        this->ScaleEnabled)
      {
        camera->GetViewPlaneNormal(vpn);
        this->Scale(prevPickPoint, pickPoint, e[0], e[1], vpn);
      }
      else if (this->InteractionState == vtkImplicitPlaneRepresentation::Rotating)
      {
        camera->GetViewPlaneNormal(vpn);
        this->Rotate(e[0], e[1], prevPickPoint, pickPoint, vpn);
      }
      else if (this->InteractionState == MultiPortSphereRegionRepresentation::RotatingArrow1)
      {
        getDirection(mArrows[0], vpn);
        this->RotateCone(e[0], e[1], prevPickPoint, pickPoint, vpn);
      }
      else if (this->InteractionState == MultiPortSphereRegionRepresentation::RotatingArrow2)
      {
        getDirection(mArrows[1], vpn);
        this->RotateCone(e[0], e[1], prevPickPoint, pickPoint, vpn);
      }
      else if (this->InteractionState == MultiPortSphereRegionRepresentation::RotatingArrow3)
      {
        getDirection(mArrows[2], vpn);
        this->RotateCone(e[0], e[1], prevPickPoint, pickPoint, vpn);
      }
      else if (this->InteractionState == vtkImplicitPlaneRepresentation::Outside &&
        this->LockNormalToCamera)
      {
        this->SetNormalToCamera();
      }

      this->LastEventPosition[0] = e[0];
      this->LastEventPosition[1] = e[1];
      this->LastEventPosition[2] = 0.0;
    }

    /** \brief returns the bounds
    */
    double *MultiPortSphereRegionRepresentation::GetBounds()
    {
      BuildRepresentation();
      BoundingBox->SetBounds(OutlineActor->GetBounds());
      BoundingBox->AddBounds(CutActor->GetBounds());
      BoundingBox->AddBounds(EdgesActor->GetBounds());
      BoundingBox->AddBounds(ConeActor->GetBounds());
      BoundingBox->AddBounds(LineActor->GetBounds());
      BoundingBox->AddBounds(ConeActor2->GetBounds());
      BoundingBox->AddBounds(LineActor2->GetBounds());
      for (int i(0); i<3; ++i)
      {
        BoundingBox->AddBounds(mArrows[i].tubeActor()->GetBounds());
        BoundingBox->AddBounds(mArrows[i].mThreshActor->GetBounds());
      }
      BoundingBox->AddBounds(SphereActor->GetBounds());
      // inforce at least twice the size of the ball
      {
        double r = Sphere->GetRadius();
        auto*  o = Plane->GetOrigin();
        double b[6];
        for (int i(0); i<3; ++i)
        {
          b[2*i] = o[i] - 2*r;
          b[2*i+1] = o[i] + 2*r;
        }
        BoundingBox->AddBounds(b);
      }
      return BoundingBox->GetBounds();
    }

    /**
    * \brief returns the actors
    * 
    * @param pc Output to get the actors
    */
    void MultiPortSphereRegionRepresentation::GetActors(vtkPropCollection *pc)
    {
      this->OutlineActor->GetActors(pc);
      this->CutActor->GetActors(pc);
      this->EdgesActor->GetActors(pc);
      this->ConeActor->GetActors(pc);
      this->LineActor->GetActors(pc);
      this->ConeActor2->GetActors(pc);
      this->LineActor2->GetActors(pc);
      for (int i(0); i<3; ++i)
      {
        mArrows[i].tubeActor()->GetActors(pc);
        mArrows[i].lineActor()->GetActors(pc);
        mArrows[i].mThreshActor->GetActors(pc);
      }
      this->SphereActor->GetActors(pc);
    }

    /** \brief Release graphics resources and ask components to release their own resources
    */
    void MultiPortSphereRegionRepresentation::ReleaseGraphicsResources(vtkWindow *w)
    {
      this->OutlineActor->ReleaseGraphicsResources(w);
      this->CutActor->ReleaseGraphicsResources(w);
      this->EdgesActor->ReleaseGraphicsResources(w);
      this->ConeActor->ReleaseGraphicsResources(w);
      this->LineActor->ReleaseGraphicsResources(w);
      this->ConeActor2->ReleaseGraphicsResources(w);
      this->LineActor2->ReleaseGraphicsResources(w);
      for (int i(0); i<3; ++i)
      {
        mArrows[i].tubeActor()->ReleaseGraphicsResources(w);
        mArrows[i].lineActor()->ReleaseGraphicsResources(w);
        mArrows[i].mThreshActor->ReleaseGraphicsResources(w);
      }
      this->SphereActor->ReleaseGraphicsResources(w);
    }

    // Support the standard render methods. 
    int MultiPortSphereRegionRepresentation::RenderOpaqueGeometry(vtkViewport *v)
    {
      int count = 0;
      this->BuildRepresentation();
      if (mRenderHandles)
      {
        count += this->OutlineActor->RenderOpaqueGeometry(v);
        count += this->EdgesActor->RenderOpaqueGeometry(v);
      }
      if (!this->LockNormalToCamera)
      {

        if (mRenderHandles)
        {
          count += this->ConeActor->RenderOpaqueGeometry(v);
          count += this->LineActor->RenderOpaqueGeometry(v);
          count += this->ConeActor2->RenderOpaqueGeometry(v);
          count += this->LineActor2->RenderOpaqueGeometry(v);
        }
        for (int i(0); i<3; ++i)
        {
          if (mPortActivity[i])
          {
            count += mArrows[i].tubeActor()->RenderOpaqueGeometry(v);
            count += mArrows[i].lineActor()->RenderOpaqueGeometry(v);
            count += mArrows[i].mThreshActor->RenderOpaqueGeometry(v);
          }
        }
        count += this->SphereActor->RenderOpaqueGeometry(v);
      }
      if (this->DrawPlane && mRenderHandles)
      {
        count += this->CutActor->RenderOpaqueGeometry(v);
      }

      return count;
    }

    // Support the standard render methods. 
    int MultiPortSphereRegionRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport *v)
    {
      int count = 0;
      this->BuildRepresentation();
      if (mRenderHandles)
      {
        count += this->OutlineActor->RenderTranslucentPolygonalGeometry(v);
        count += this->EdgesActor->RenderTranslucentPolygonalGeometry(v);
      }
      if (!this->LockNormalToCamera)
      {
        if (mRenderHandles)
        {
          count += this->ConeActor->RenderTranslucentPolygonalGeometry(v);
          count += this->LineActor->RenderTranslucentPolygonalGeometry(v);
          count += this->ConeActor2->RenderTranslucentPolygonalGeometry(v);
          count += this->LineActor2->RenderTranslucentPolygonalGeometry(v);
        }
        for (int i(0); i<3; ++i)
        {
          if (mPortActivity[i])
          {
            count += mArrows[i].tubeActor()->RenderTranslucentPolygonalGeometry(v);
            count += mArrows[i].lineActor()->RenderTranslucentPolygonalGeometry(v);
            count += mArrows[i].mThreshActor->RenderTranslucentPolygonalGeometry(v);
          }
        }
        count += this->SphereActor->RenderTranslucentPolygonalGeometry(v);
      }
      if (this->DrawPlane && mRenderHandles)
      {
        count += this->CutActor->RenderTranslucentPolygonalGeometry(v);
      }

      return count;
    }

    // Support the standard render methods. 
    int MultiPortSphereRegionRepresentation::HasTranslucentPolygonalGeometry()
    {
      int result = 0;
      if (mRenderHandles)
      {
        result |= this->OutlineActor->HasTranslucentPolygonalGeometry();
        result |= this->EdgesActor->HasTranslucentPolygonalGeometry();
      }
      if (!this->LockNormalToCamera)
      {
        if (mRenderHandles)
        {
          result |= this->ConeActor->HasTranslucentPolygonalGeometry();
          result |= this->LineActor->HasTranslucentPolygonalGeometry();
          result |= this->ConeActor2->HasTranslucentPolygonalGeometry();
          result |= this->LineActor2->HasTranslucentPolygonalGeometry();
        }
        for (int i(0); i<3; ++i)
        {
          if (mPortActivity[i])
          {
            result |= mArrows[i].tubeActor()->HasTranslucentPolygonalGeometry();
            result |= mArrows[i].lineActor()->HasTranslucentPolygonalGeometry();
            result |= mArrows[i].mThreshActor->HasTranslucentPolygonalGeometry();
          }
        }
        result |= this->SphereActor->HasTranslucentPolygonalGeometry();
      }
      if (this->DrawPlane && mRenderHandles)
      {
        result |= this->CutActor->HasTranslucentPolygonalGeometry();
      }

      return result;
    }

    //----------------------------------------------------------------------------
    // used to print the state
    //----------------------------------------------------------------------------
    void MultiPortSphereRegionRepresentation::PrintSelf(ostream& os, vtkIndent indent)
    {
      this->Superclass::PrintSelf(os, indent);

      if (this->NormalProperty)
      {
        os << indent << "Normal Property: " << this->NormalProperty << "\n";
      }
      else
      {
        os << indent << "Normal Property: (none)\n";
      }
      if (this->SelectedNormalProperty)
      {
        os << indent << "Selected Normal Property: "
          << this->SelectedNormalProperty << "\n";
      }
      else
      {
        os << indent << "Selected Normal Property: (none)\n";
      }

      if (this->mpArrowProperty)
      {
        os << indent << "Cone Property: " << this->mpArrowProperty << "\n";
      }
      else
      {
        os << indent << "Cone Property: (none)\n";
      }
      if (this->mpSelectedArrowProperty)
      {
        os << indent << "Selected Cone Property: "
          << this->mpSelectedArrowProperty << "\n";
      }
      else
      {
        os << indent << "Selected Cone Property: (none)\n";
      }

      if (this->PlaneProperty)
      {
        os << indent << "Plane Property: " << this->PlaneProperty << "\n";
      }
      else
      {
        os << indent << "Plane Property: (none)\n";
      }
      if (this->SelectedPlaneProperty)
      {
        os << indent << "Selected Plane Property: "
          << this->SelectedPlaneProperty << "\n";
      }
      else
      {
        os << indent << "Selected Plane Property: (none)\n";
      }

      if (this->OutlineProperty)
      {
        os << indent << "Outline Property: " << this->OutlineProperty << "\n";
      }
      else
      {
        os << indent << "Outline Property: (none)\n";
      }
      if (this->SelectedOutlineProperty)
      {
        os << indent << "Selected Outline Property: "
          << this->SelectedOutlineProperty << "\n";
      }
      else
      {
        os << indent << "Selected Outline Property: (none)\n";
      }

      if (this->EdgesProperty)
      {
        os << indent << "Edges Property: " << this->EdgesProperty << "\n";
      }
      else
      {
        os << indent << "Edges Property: (none)\n";
      }

      os << indent << "Normal To X Axis: "
        << (this->NormalToXAxis ? "On" : "Off") << "\n";
      os << indent << "Normal To Y Axis: "
        << (this->NormalToYAxis ? "On" : "Off") << "\n";
      os << indent << "Normal To Z Axis: "
        << (this->NormalToZAxis ? "On" : "Off") << "\n";
      os << indent << "Lock Normal To Camera: "
        << (this->LockNormalToCamera ? "On" : "Off") << "\n";

      os << indent << "Widget Bounds: " << this->WidgetBounds[0] << ", "
        << this->WidgetBounds[1] << ", "
        << this->WidgetBounds[2] << ", "
        << this->WidgetBounds[3] << ", "
        << this->WidgetBounds[4] << ", "
        << this->WidgetBounds[5] << "\n";

      os << indent << "Tubing: " << (this->Tubing ? "On" : "Off") << "\n";
      os << indent << "Outline Translation: "
        << (this->OutlineTranslation ? "On" : "Off") << "\n";
      os << indent << "Outside Bounds: "
        << (this->OutsideBounds ? "On" : "Off") << "\n";
      os << indent << "Constrain to Widget Bounds: "
        << (this->ConstrainToWidgetBounds ? "On" : "Off") << "\n";
      os << indent << "Scale Enabled: "
        << (this->ScaleEnabled ? "On" : "Off") << "\n";
      os << indent << "Draw Plane: " << (this->DrawPlane ? "On" : "Off") << "\n";
      os << indent << "Bump Distance: " << this->BumpDistance << "\n";

      os << indent << "Representation State: ";
      switch (this->RepresentationState)
      {
        case Outside:
          os << "Outside\n";
          break;
        case Moving:
          os << "Moving\n";
          break;
        case MovingOutline:
          os << "MovingOutline\n";
          break;
        case MovingOrigin:
          os << "MovingOrigin\n";
          break;
        case Rotating:
          os << "Rotating\n";
          break;
        case Pushing:
          os << "Pushing\n";
          break;
        case Scaling:
          os << "Scaling\n";
          break;
        case RotatingArrow1:
          os << "RotatingCone3\n";
          break;
        case RotatingArrow2:
          os << "RotatingCone4\n";
          break;

        case RotatingArrow3:
          os << "RotatingCone5\n";
          break;


      }

      // this->InteractionState is printed in superclass
      // this is commented to avoid PrintSelf errors
    }

    // \brief Creates the default properties for the Objects
    void MultiPortSphereRegionRepresentation::CreateDefaultProperties()
    {
      // Normal properties
      this->NormalProperty = vtkProperty::New();
      this->NormalProperty->SetColor(1, 1, 1);
      this->NormalProperty->SetLineWidth(2);

      this->SelectedNormalProperty = vtkProperty::New();
      this->SelectedNormalProperty->SetColor(1, 0, 0);
      this->SelectedNormalProperty->SetLineWidth(2);

      // ArrowProperties
      this->mpArrowProperty = make_vtk<vtkProperty>();
      this->mpArrowProperty->SetColor(0, 1, 0);
      this->mpArrowProperty->SetLineWidth(2);
      this->mpArrowProperty->SetOpacity(0.5);

      this->mpSelectedArrowProperty = make_vtk<vtkProperty>();
      this->mpSelectedArrowProperty->SetColor(0, 1, 0);
      this->mpSelectedArrowProperty->SetOpacity(0.5);
      
      // Plane properties
      this->PlaneProperty = vtkProperty::New();
      this->PlaneProperty->SetAmbient(1.0);
      this->PlaneProperty->SetAmbientColor(1.0, 1.0, 1.0);
      this->PlaneProperty->SetOpacity(0.5);
      this->CutActor->SetProperty(this->PlaneProperty);

      this->SelectedPlaneProperty = vtkProperty::New();
      this->SelectedPlaneProperty->SetAmbient(1.0);
      this->SelectedPlaneProperty->SetAmbientColor(0.0, 1.0, 0.0);
      this->SelectedPlaneProperty->SetOpacity(0.25);

      // Outline properties
      this->OutlineProperty = vtkProperty::New();
      this->OutlineProperty->SetAmbient(1.0);
      this->OutlineProperty->SetAmbientColor(1.0, 1.0, 1.0);

      this->SelectedOutlineProperty = vtkProperty::New();
      this->SelectedOutlineProperty->SetAmbient(1.0);
      this->SelectedOutlineProperty->SetAmbientColor(0.0, 1.0, 0.0);

      // Edge property
      this->EdgesProperty = vtkProperty::New();
      this->EdgesProperty->SetAmbient(1.0);
      this->EdgesProperty->SetAmbientColor(1.0, 1.0, 1.0);
    }

    // \brief Hightlights the selected Objects
    void MultiPortSphereRegionRepresentation::HighlightNormal(int highlight)
    {
      if (highlight)
      {
        this->LineActor->SetProperty(this->SelectedNormalProperty);
        this->ConeActor->SetProperty(this->SelectedNormalProperty);
        this->LineActor2->SetProperty(this->SelectedNormalProperty);
        this->ConeActor2->SetProperty(this->SelectedNormalProperty);
        for (int i(0); i<3; ++i)
        {
          mArrows[i].tubeActor()->SetProperty(this->mpSelectedArrowProperty);
        }
        this->SphereActor->SetProperty(this->SelectedNormalProperty);
      }
      else
      {
        this->LineActor->SetProperty(this->NormalProperty);
        this->ConeActor->SetProperty(this->NormalProperty);
        this->LineActor2->SetProperty(this->NormalProperty);
        this->ConeActor2->SetProperty(this->NormalProperty);
        for (int i(0); i<3; ++i)
        {
          mArrows[i].tubeActor()->SetProperty(this->mpArrowProperty);
        }
        this->SphereActor->SetProperty(this->NormalProperty);
      }
    }

    /**
    */
    void MultiPortSphereRegionRepresentation::setPortActivity(const Vec3i& p)
    {
      mPortActivity = p;
      this->Modified();
    }

    /**
    */
    void MultiPortSphereRegionRepresentation::setHandleVisibility(bool active)
    {
      if (active)
      {
        mRenderHandles = true;
      }
      else
      {
        mRenderHandles = false;
        auto prop = make_vtk<vtkProperty>();
        prop->DeepCopy(SphereActor->GetProperty());
        prop->SetOpacity(0.5);
        SphereActor->SetProperty(prop);
        mpArrowProperty->SetOpacity(1.0);
      }
      Modified();
    }
  }
}

namespace
{
  using namespace gris::muk;
  /**
  */
  void setMatrix(vtkTransform* trafo, const Vec3d& dir, double T[3])
  {
    auto M = make_vtk<vtkMatrix4x4>();
    M->Identity();
    const auto n  = Vec3d(0,0,1);
    const auto v = dir.cross(n);
    const double phi = -std::acos(n.dot(dir)) * 180/gris::M_Pi;
    trafo->Translate(T);
    trafo->RotateWXYZ(phi, v.data());
  }
}