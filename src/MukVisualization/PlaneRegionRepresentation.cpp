#include "private/muk.pch"
#include "PlaneRegionRepresentation.h"

#include "ArrowTrajectory.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/gris_math.h"
#include "MukCommon/polygon_3d_tools.h"
#include "MukCommon/vtk_tools.h"

#include <vtkAssemblyPaths.h>
#include <vtkBox.h>
#include <vtkCamera.h>
#include <vtkCutter.h>
#include <vtkInteractorObserver.h>
#include <vtkLine.h>
#include <vtkPropPicker.h>
#include <vtkTransform.h>

#include <algorithm>

namespace
{
  using namespace gris;
  using namespace gris::muk;
  const double mArrowScaling = 1.2;

  void adjustBounds(Vec3d& min, Vec3d& max);
}

namespace gris
{
  namespace muk
  {
    vtkStandardNewMacro(PlaneRegionRepresentation);

    /**
    */
    PlaneRegionRepresentation::DirectionBall::DirectionBall()
    {
      sphere       = make_vtk<vtkSphereSource>();
      sphereMapper = make_vtk<vtkPolyDataMapper>();
      sphereActor  = make_vtk<vtkActor>();
      //sphere->SetRadius(1.0);
      sphereMapper->SetInputConnection(sphere->GetOutputPort());
      sphereActor->SetMapper(sphereMapper);

      lines       = make_vtk<vtkPolyData>();
      linesMapper = make_vtk<vtkPolyDataMapper>();
      linesActor  = make_vtk<vtkActor>();
      linesMapper->SetInputData(lines);
      linesActor->SetMapper(linesMapper);
      linesActor->GetProperty()->SetLineWidth(2.0);

      auto points    = make_vtk<vtkPoints>();
      auto linesCell = make_vtk<vtkCellArray>();
      auto line      = make_vtk<vtkLine>();
      points->InsertNextPoint(0,0,0);
      points->InsertNextPoint(1,0,0);
      line->GetPointIds()->SetNumberOfIds(2);
      line->GetPointIds()->SetId(0,0);
      line->GetPointIds()->SetId(1,1);
      linesCell->InsertNextCell(line);
      lines->SetPoints(points);
      lines->SetLines(linesCell);
    }

    /** \brief Default Initialisation of the Representation
    */
    PlaneRegionRepresentation::PlaneRegionRepresentation()
    {
      //Sphere->SetRadius(1.0);
      Sphere->SetPhiResolution(30);
      Sphere->SetThetaResolution(30);
      Picker->AddPickList(mDirBall.sphereActor); // Manage the picking stuff of the Arrows
      mDirBall.sphere->SetCenter(1,0,0);
      
      CreateDefaultProperties();
    }

    /** \brief Default destruction of the Representation
    */
    PlaneRegionRepresentation::~PlaneRegionRepresentation() 
    {
    };

    /**
    */
    void PlaneRegionRepresentation::SetInteractionState(int state)
    {
      InteractionState = state;
    }

    /**
    */
    void PlaneRegionRepresentation::centralize(const Vec3d& p)
    {
      auto* b = GetBounds();
      const auto extent = 0.5*Vec3d(b[1]-b[0], b[3]-b[2], b[5]-b[4]);
      double bnew[6];
      for (int i(0); i<3; ++i)
      {
        bnew[2*i]   = p[i] - extent[i];
        bnew[2*i+1] = p[i] + extent[i];
      }
      const auto v = Vec3d(mDirBall.sphere->GetCenter()) - Vec3d(Sphere->GetCenter());
      PlaceWidget(bnew);
      Plane->SetOrigin(const_cast<Vec3d&>(p).data());
      mDirBall.sphere->SetCenter( (p+v).data() );
    }

    /**
    */
    Vec3d PlaneRegionRepresentation::getDirectionAncor() const 
    {
      return Vec3d(mDirBall.sphere->GetCenter());
    }

    /**
    */
    std::vector<Vec3d> PlaneRegionRepresentation::getBoundingPolygon() const
    {
      std::vector<Vec3d> res;
      auto* poly = Cutter->GetOutput();
      const auto N = poly->GetNumberOfPoints();
      for (vtkIdType i(0); i<N; ++i)
      {
        auto next = Vec3d(poly->GetPoint(i));
        res.push_back(next);
      }
      return res;
    }

    /**
    */
    void PlaneRegionRepresentation::setPlane(const Vec3d& p, const Vec3d& n)
    {
      Plane->SetOrigin(p.x(), p.y(), p.z());
      Plane->SetNormal(n.x(), n.y(), n.z());
      Modified();
      BuildRepresentation();
    }

    /**
    */
    void PlaneRegionRepresentation::setDirectionAncor(const Vec3d& p)
    {
      mDirBall.sphere->SetCenter(p.x(), p.y(), p.z());
      Modified();
      BuildRepresentation();
    }

    /**
    */
    void PlaneRegionRepresentation::setBounds(const std::vector<Vec3d>& input)
    {
      Vec3d mini(std::numeric_limits<double>::max());
      Vec3d maxi(-std::numeric_limits<double>::max());
      for (const auto& p : input)
      {
        for (int i(0); i<3; ++i)
        {
          mini[i] = std::min(mini[i], p[i]);
          maxi[i] = std::max(maxi[i], p[i]);
        }
      }
      // naturally, there is a largest and a singular direction, find those and adapt
      adjustBounds(mini, maxi);
      // adjust the bounding box
      double d[6] = { mini.x(), maxi.x(), mini.y(), maxi.y(), mini.z(), maxi.z() };
      PlaceWidget(d);
    }


    /** *\brief checks wich Interactionstate is activated
    *
    * @param X x-coordinate of the mouse
    * @param Y y-coordinate of the mouse
    * 
    */
    int PlaneRegionRepresentation::ComputeInteractionState(int X, int Y, int modify)
    {
      // See if anything has been selected
      vtkAssemblyPath* path = GetAssemblyPath(X, Y, 0., Picker);
      if (path == nullptr) // Not picking this widget
      {
        SetRepresentationState(enOutside);
        InteractionState = enOutside;
        return InteractionState;
      }
      // Something picked, continue
      ValidPick = 1;
      // Depending on the interaction state (set by the widget) we modify
      // this state based on what is picked.
      if (InteractionState == enMoving)
      {
        vtkProp *prop = path->GetFirstNode()->GetViewProp();
        int en;
        {
          if ((prop == ConeActor || prop == LineActor || prop == ConeActor2 || prop == LineActor2))
            en = enRotating;
          else if (prop == mDirBall.sphereActor.Get())
            en = enMovingDirectionBall;
          else if (prop == CutActor)
            if (LockNormalToCamera) // Allow camera to work
              en = enOutside;
            else
              en = enPushing;
          else if (prop == SphereActor)
            en = enMovingOrigin;
          else
            if (OutlineTranslation)
              en = enMovingOutline;
            else
              en = enOutside;
        }
        InteractionState = en;
        SetRepresentationState(en);
      }
      else if (InteractionState != enScaling) // We may add a condition to allow the camera to work IO scaling
      {
        InteractionState = enOutside;
      }
      return InteractionState;
    }

    /** \brief Builds the the representation based on the normal and origin of the plane

      typical reinitialization procedure after more or less any changing event
    */
    void PlaneRegionRepresentation::BuildRepresentation()
    {
      if (!Renderer || !Renderer->GetRenderWindow())
      {
        return;
      }
      vtkInformation *info = GetPropertyKeys();
      OutlineActor->SetPropertyKeys(info);
      CutActor->SetPropertyKeys(info);
      EdgesActor->SetPropertyKeys(info);
      ConeActor->SetPropertyKeys(info);
      LineActor->SetPropertyKeys(info);
      ConeActor2->SetPropertyKeys(info);
      LineActor2->SetPropertyKeys(info);
      SphereActor->SetPropertyKeys(info);
      mDirBall.sphereActor->SetPropertyKeys(info);
      mDirBall.linesActor->SetPropertyKeys(info);

      if (mArrows.getActor()->GetVisibility()
        && GetMTime() > BuildTime)
      {
        Outline->Update();
        Cutter->Update();
        const auto origin = Vec3d(GetOrigin());
        const auto normal = Vec3d(GetNormal());
        const auto ancor = getDirectionAncor();
        const auto polygon = getBoundingPolygon();
        std::vector<MukState> result;
        // compute maximum distance to a point on the polygon
        double dist = 0;
        for (const auto& p : polygon)
        {
          const auto d = (origin-p).norm();
          if (d>dist)
            dist = d;
        }
        // create the polygon
        Polygon3D poly;
        poly.create(polygon);
        // traverse a rectangular grid on the plane with the given resolution and the width 'dist'
        // compute for each grid point, if it is inside the polygon and if so, create a new state from it
        const auto rand1 = Vec3d(1, 0, 0);
        const auto rand2 = Vec3d(0, 1, 0);
        auto v1 = rand1.cross(normal).normalized();
        if (v1.hasNaN())
          v1 = rand2.cross(normal).normalized();
        const auto v2 = v1.cross(normal).normalized();
        double res = 1.0;
        const auto N = std::max(1, static_cast<int>(dist / res + 0.5));
        for (int i(-N); i<=N; ++i)
        {
          for (int j(-N); j<=N; ++j)
          {
            MukState next;
            next.coords = origin + i*res*v1 + j*res*v2;
            if (poly.isInside(next.coords))
            {
              // TODO_: -1 is a hack, because controllers / models cannot distinguish properly betweeen start and goal region
              next.tangent = (-1)*(ancor- next.coords).normalized();
              result.push_back(next);
            }
          }
        }
        mArrows.clear();
        for (const auto& state : result)
        {
          mArrows.addArrow(Arrow::create(state.coords, state.tangent));
        }
        mArrows.transform();

        SizeHandles();
        BuildTime.Modified();
      }
      else
      {
        if (GetMTime() > BuildTime ||
          Plane->GetMTime() > BuildTime ||
          Renderer->GetRenderWindow()->GetMTime() > BuildTime)
        {
          // ==========
          // first some base class vtk procedure
          // ==========
          double *origin = Plane->GetOrigin();
          double *normal = Plane->GetNormal();
          double bounds[6];
          std::copy(WidgetBounds, WidgetBounds + 6, bounds);
          // Center of the Cones | vector of the Lines of the Arrows
          double p2[3];
          // Checks if the Origin traverse the Bounds
          if (!OutsideBounds)
          {
            // restrict the origin inside InitialBounds
            double *ibounds = InitialBounds;
            for (int i = 0; i < 3; i++)
            {
              if (origin[i] < ibounds[2 * i])
                origin[i] = ibounds[2 * i];
              else if (origin[i] > ibounds[2 * i + 1])
                origin[i] = ibounds[2 * i + 1];
            }
          }

          if (ConstrainToWidgetBounds)
          {
            if (!OutsideBounds)
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
            double offset = Box->GetLength() * 0.02;
            for (int i = 0; i < 3; ++i)
            {
              bounds[2 * i] = vtkMath::Min(origin[i] - offset, WidgetBounds[2 * i]);
              bounds[2 * i + 1] = vtkMath::Max(origin[i] + offset, WidgetBounds[2 * i + 1]);
            }
          }

          Box->SetOrigin(bounds[0], bounds[2], bounds[4]);
          Box->SetSpacing((bounds[1] - bounds[0]), (bounds[3] - bounds[2]),
            (bounds[5] - bounds[4]));
          Outline->Update();

          // Setup the length of the lines
          double d = 0.5*Outline->GetOutput()->GetLength();

          p2[0] = origin[0] + d * normal[0];
          p2[1] = origin[1] + d * normal[1];
          p2[2] = origin[2] + d * normal[2];

          LineSource->SetPoint1(origin);
          LineSource->SetPoint2(p2);
          ConeSource->SetCenter(p2);
          ConeSource->SetDirection(normal);

          p2[0] = origin[0] - d * normal[0];
          p2[1] = origin[1] - d * normal[1];
          p2[2] = origin[2] - d * normal[2];

          LineSource2->SetPoint1(origin[0], origin[1], origin[2]);
          LineSource2->SetPoint2(p2);
          ConeSource2->SetCenter(p2);
          ConeSource2->SetDirection(normal[0], normal[1], normal[2]);
          // ==========
          // start of additional procedure
          // ==========
          mDirBall.lines->GetPoints()->SetPoint(0, Sphere->GetCenter());
          mDirBall.lines->GetPoints()->SetPoint(1, mDirBall.sphere->GetCenter());
          mDirBall.lines->Modified();

          // Set up the Origin
          Sphere->SetCenter(origin[0], origin[1], origin[2]);

          // Control the look of the edges
          if (Tubing)
          {
            EdgesMapper->SetInputConnection(
              EdgesTuber->GetOutputPort());
          }
          else
          {
            EdgesMapper->SetInputConnection(
              Edges->GetOutputPort());
          }
          SizeHandles();
          BuildTime.Modified();
        }
      }
    }
    /**
    * \brief Manages the the size of the Cones if Zoomed
    */
    void PlaneRegionRepresentation::SizeHandles()
    {
      // radius scales with the zoom to pick actors better (not used for cones but it may be changed here)
      double r = vtkWidgetRepresentation::SizeHandlesInPixels(1.5, Sphere->GetCenter());
      // defaultRadius for the Arrows' Cones (change to scale)
      ConeSource->SetHeight(2*r);
      ConeSource->SetRadius(r);
      ConeSource2->SetHeight(2*r);
      ConeSource2->SetRadius(r);
      Sphere->SetRadius(r);
      EdgesTuber->SetRadius(0.25*r);
      mDirBall.sphere->SetRadius(r);
    }

    //----------------------------------------------------------------------------
    /**
    */
    void PlaneRegionRepresentation::PlacePlane(double* e)
    {
      // Pick from this location.
      auto picker = make_vtk<vtkPropPicker>();
      picker->Pick(e[0], e[1], 0, Renderer);
      if (picker->GetActor() == nullptr)
        return;
      double* p1 = Plane->GetOrigin();
      double* p2 = picker->GetPickPosition();
      // Place box and origin
      {
        //Get the motion vector
        double v[3];
        v[0] = p2[0] - p1[0];
        v[1] = p2[1] - p1[1];
        v[2] = p2[2] - p1[2];

        //Translate the bounding box
        auto* origin = Box->GetOrigin();
        double oNew[3];
        oNew[0] = origin[0] + v[0];
        oNew[1] = origin[1] + v[1];
        oNew[2] = origin[2] + v[2];
        Box->SetOrigin(oNew);
        Box->GetBounds(WidgetBounds);

        //Translate the plane
        origin = Plane->GetOrigin();
        oNew[0] = origin[0] + v[0];
        oNew[1] = origin[1] + v[1];
        oNew[2] = origin[2] + v[2];
        Plane->SetOrigin(oNew);
      }
      BuildRepresentation();
    }

    //----------------------------------------------------------------------------
    /**
    */
    void PlaneRegionRepresentation::PlaceSphere(double* e)
    {
      // Pick from this location.
      auto picker = make_vtk<vtkPropPicker>();
      picker->Pick(e[0], e[1], 0, Renderer);
      if (picker->GetActor() == nullptr)
        return;
      double* pos = picker->GetPickPosition();
      // place the sphere
      mDirBall.sphere->SetCenter(pos);
      BuildRepresentation();
    }
    
    //----------------------------------------------------------------------------
    // Loop through all points and translate them
    void PlaneRegionRepresentation::TranslateOutline(double *p1, double *p2)
    {
      //Get the motion vector
      double v[3];
      v[0] = p2[0] - p1[0];
      v[1] = p2[1] - p1[1];
      v[2] = p2[2] - p1[2];

      //Translate the bounding box
      double *origin = Box->GetOrigin();
      double oNew[3];
      oNew[0] = origin[0] + v[0];
      oNew[1] = origin[1] + v[1];
      oNew[2] = origin[2] + v[2];
      Box->SetOrigin(oNew);
      Box->GetBounds(WidgetBounds);

      //Translate the plane
      origin = Plane->GetOrigin();
      oNew[0] = origin[0] + v[0];
      oNew[1] = origin[1] + v[1];
      oNew[2] = origin[2] + v[2];
      Plane->SetOrigin(oNew);

      BuildRepresentation();
    }
    
    /** \brief LockNormalToCamera will cause the normal to follow the camera's normal
    *
    * @param lock Wether Camera is locked or not
    */
    void PlaneRegionRepresentation::SetLockNormalToCamera(int lock)
    {
      vtkDebugMacro(<< GetClassName() << " (" << this << "): setting " << LockNormalToCamera << " to " << lock);
      if (lock == LockNormalToCamera)
      {
        return;
      }
      if (lock)
      {
        Picker->DeletePickList(LineActor);
        Picker->DeletePickList(ConeActor);
        Picker->DeletePickList(LineActor2);
        Picker->DeletePickList(ConeActor2);
        Picker->DeletePickList(mDirBall.sphereActor);
        Picker->DeletePickList(SphereActor);
        SetNormalToCamera();
      }
      else
      {
        Picker->AddPickList(LineActor);
        Picker->AddPickList(ConeActor);
        Picker->AddPickList(LineActor2);
        Picker->AddPickList(ConeActor2);
        Picker->AddPickList(mDirBall.sphereActor);
        Picker->AddPickList(SphereActor);
      }
      LockNormalToCamera = lock;
      Modified();
    }

    /**
    * \brief Highights the Objects wether it's their Representationstate
    * 
    * @param state The new Representationstate
    */
    void PlaneRegionRepresentation::SetRepresentationState(int state)
    {
      if (RepresentationState == state)
      {
        return;
      }

      // Clamp the state
      state = state < enOutside ? enOutside : (state > enScaling ? enScaling : state);
      RepresentationState = state;
      Modified();

      switch (state)
      {
        case enRotating:
        case enPushing:
          HighlightNormal(1);
          HighlightPlane(1);
          break;
        case enMovingOrigin:
          HighlightNormal(1);
          break;
        case enMovingOutline:
          HighlightOutline(1);
          break;
        case enScaling:
          if (ScaleEnabled)
          {
            HighlightNormal(1);
            HighlightPlane(1);
            HighlightOutline(1);
          }
          break;
        case enMovingDirectionBall:
          HighlightDirectionBall(1);
          break;
        default:
          HighlightNormal(0);
          HighlightPlane(0);
          HighlightOutline(0);
      };
    }

    /**
    *  \brief Starts calculation for the Interactions
    *
    * @param e[2] Mouseposition on screen
    */
    void PlaneRegionRepresentation::WidgetInteraction(double e[2])
    {
      // Do different things depending on state
      double focalPoint[4], pickPoint[4], prevPickPoint[4];
      double z, vpn[3];

      vtkCamera *camera = Renderer->GetActiveCamera();
      if (!camera)
      {
        return;
      }

      // Compute the two points defining the motion vector
      double pos[3];
      Picker->GetPickPosition(pos);
      vtkInteractorObserver::ComputeWorldToDisplay(Renderer, pos[0], pos[1], pos[2], focalPoint);
      z = focalPoint[2];
      vtkInteractorObserver::ComputeDisplayToWorld(Renderer, LastEventPosition[0], LastEventPosition[1], z, prevPickPoint);
      vtkInteractorObserver::ComputeDisplayToWorld(Renderer, e[0], e[1], z, pickPoint);
      switch (InteractionState)
      {
        case enMovingOutline:
          TranslateOutline(prevPickPoint, pickPoint);
          break;
        case enMovingOrigin:
          TranslateOrigin(prevPickPoint, pickPoint);
          break;
        case enPushing:
          Push(prevPickPoint, pickPoint);
          break;
        case enScaling:
          if (ScaleEnabled)
          {
            camera->GetViewPlaneNormal(vpn);
            Scale(prevPickPoint, pickPoint, e[0], e[1]);
          }
          break;
        case enRotating:
          camera->GetViewPlaneNormal(vpn);
          Rotate(e[0], e[1], prevPickPoint, pickPoint, vpn);
          break;
        case enMovingDirectionBall:
          TranslateDirectionBall(prevPickPoint, pickPoint);
          break;
        case enPlacePlane:
          PlacePlane(e);
          break;
        case enPlaceAncor:
          PlaceSphere(e);
          break;
        case enOutside:
          if (LockNormalToCamera)
            SetNormalToCamera();
          break;
      };

      LastEventPosition[0] = e[0];
      LastEventPosition[1] = e[1];
      LastEventPosition[2] = 0.0;
    }

    /**
    */
    void PlaneRegionRepresentation::TranslateDirectionBall(double *p1, double *p2)
    {
      //Get the motion vector
      double v[3];
      v[0] = p2[0] - p1[0];
      v[1] = p2[1] - p1[1];
      v[2] = p2[2] - p1[2];

      //Translate the direction ball sphere
      double *origin = mDirBall.sphere->GetCenter();
      double oNew[3];
      oNew[0] = origin[0] + v[0];
      oNew[1] = origin[1] + v[1];
      oNew[2] = origin[2] + v[2];
      mDirBall.sphere->SetCenter(oNew);
      mDirBall.lines->GetPoints()->SetPoint(1,oNew);
      mDirBall.lines->Modified();
      
      BuildRepresentation();
    }

    /** \brief returns the bounds
    */
    double* PlaneRegionRepresentation::GetBounds()
    {
      BuildRepresentation();
      BoundingBox->SetBounds(OutlineActor->GetBounds());
      BoundingBox->AddBounds(CutActor->GetBounds());
      BoundingBox->AddBounds(EdgesActor->GetBounds());
      BoundingBox->AddBounds(ConeActor->GetBounds());
      BoundingBox->AddBounds(LineActor->GetBounds());
      BoundingBox->AddBounds(ConeActor2->GetBounds());
      BoundingBox->AddBounds(LineActor2->GetBounds());
      BoundingBox->AddBounds(mDirBall.sphereActor->GetBounds());
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
    void PlaneRegionRepresentation::GetActors(vtkPropCollection *pc)
    {
      OutlineActor->GetActors(pc);
      CutActor->GetActors(pc);
      EdgesActor->GetActors(pc);
      ConeActor->GetActors(pc);
      LineActor->GetActors(pc);
      ConeActor2->GetActors(pc);
      LineActor2->GetActors(pc);
      mDirBall.sphereActor->GetActors(pc);
      mDirBall.linesActor->GetActors(pc);
      SphereActor->GetActors(pc);
      mArrows.getActor()->GetActors(pc);
    }

    /** \brief Release graphics resources and ask components to release their own resources
    */
    void PlaneRegionRepresentation::ReleaseGraphicsResources(vtkWindow *w)
    {
      OutlineActor->ReleaseGraphicsResources(w);
      CutActor->ReleaseGraphicsResources(w);
      EdgesActor->ReleaseGraphicsResources(w);
      ConeActor->ReleaseGraphicsResources(w);
      LineActor->ReleaseGraphicsResources(w);
      ConeActor2->ReleaseGraphicsResources(w);
      LineActor2->ReleaseGraphicsResources(w);
      mDirBall.sphereActor->ReleaseGraphicsResources(w);
      mDirBall.linesActor->ReleaseGraphicsResources(w);
      SphereActor->ReleaseGraphicsResources(w);
      mArrows.getActor()->ReleaseGraphicsResources(w);
    }

    // Support the standard render methods. 
    int PlaneRegionRepresentation::RenderOpaqueGeometry(vtkViewport *v)
    {
      int count = 0;
      BuildRepresentation();
      if (OutlineActor->GetVisibility())
        count += OutlineActor->RenderOpaqueGeometry(v);
      if (EdgesActor->GetVisibility())
        count += EdgesActor->RenderOpaqueGeometry(v);
      if (mArrows.getActor()->GetVisibility())
        count += mArrows.getActor()->RenderOpaqueGeometry(v);
      if (!LockNormalToCamera)
      {

        if (ConeActor->GetVisibility())
        {
          count += ConeActor->RenderOpaqueGeometry(v);
          count += LineActor->RenderOpaqueGeometry(v);
          count += ConeActor2->RenderOpaqueGeometry(v);
          count += LineActor2->RenderOpaqueGeometry(v);
        }
        if (SphereActor->GetVisibility())
        {
          count += mDirBall.sphereActor->RenderOpaqueGeometry(v);
          count += mDirBall.linesActor->RenderOpaqueGeometry(v);
          count += SphereActor->RenderOpaqueGeometry(v);
        }
      }
      if (DrawPlane && CutActor->GetVisibility())
      {
        count += CutActor->RenderOpaqueGeometry(v);
      }
      return count;
    }

    // Support the standard render methods. 
    int PlaneRegionRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport *v)
    {
      int count = 0;
      BuildRepresentation();
      if (OutlineActor->GetVisibility())
      {
        count += OutlineActor->RenderTranslucentPolygonalGeometry(v);
        count += EdgesActor->RenderTranslucentPolygonalGeometry(v);
      }
      if (mArrows.getActor()->GetVisibility())
        count += mArrows.getActor()->RenderTranslucentPolygonalGeometry(v);
      
      if (!LockNormalToCamera)
      {
        if (ConeActor->GetVisibility())
        {
          count += ConeActor->RenderTranslucentPolygonalGeometry(v);
          count += LineActor->RenderTranslucentPolygonalGeometry(v);
          count += ConeActor2->RenderTranslucentPolygonalGeometry(v);
          count += LineActor2->RenderTranslucentPolygonalGeometry(v);
        }
        if (SphereActor->GetVisibility())
        {
          count += mDirBall.sphereActor->RenderTranslucentPolygonalGeometry(v);
          count += mDirBall.linesActor->RenderTranslucentPolygonalGeometry(v);
          count += SphereActor->RenderTranslucentPolygonalGeometry(v);
        }
      }
      if (DrawPlane && CutActor->GetVisibility())
      {
        count += CutActor->RenderTranslucentPolygonalGeometry(v);
      }

      return count;
    }

    // Support the standard render methods. 
    int PlaneRegionRepresentation::HasTranslucentPolygonalGeometry()
    {
      int result = 0;
      if (OutlineActor->GetVisibility())
      {
        result |= OutlineActor->HasTranslucentPolygonalGeometry();
        result |= EdgesActor->HasTranslucentPolygonalGeometry();
      }
      if (mArrows.getActor()->GetVisibility())
        result |= mArrows.getActor()->HasTranslucentPolygonalGeometry();
      if (!LockNormalToCamera)
      {
        if (ConeActor->GetVisibility())
        {
          result |= ConeActor->HasTranslucentPolygonalGeometry();
          result |= LineActor->HasTranslucentPolygonalGeometry();
          result |= ConeActor2->HasTranslucentPolygonalGeometry();
          result |= LineActor2->HasTranslucentPolygonalGeometry();
        }
        if (SphereActor->GetVisibility())
        {
          result |= mDirBall.sphereActor->HasTranslucentPolygonalGeometry();
          result |= mDirBall.linesActor->HasTranslucentPolygonalGeometry();
          result |= SphereActor->HasTranslucentPolygonalGeometry();
        }
      }
      if (DrawPlane && CutActor->GetVisibility())
      {
        result |= CutActor->HasTranslucentPolygonalGeometry();
      }

      return result;
    }

    //----------------------------------------------------------------------------
    // used to print the state
    //----------------------------------------------------------------------------
    void PlaneRegionRepresentation::PrintSelf(ostream& os, vtkIndent indent)
    {
      Superclass::PrintSelf(os, indent);

      if (NormalProperty)
      {
        os << indent << "Normal Property: " << NormalProperty << "\n";
      }
      else
      {
        os << indent << "Normal Property: (none)\n";
      }
      if (SelectedNormalProperty)
      {
        os << indent << "Selected Normal Property: " << SelectedNormalProperty << "\n";
      }
      else
      {
        os << indent << "Selected Normal Property: (none)\n";
      }

      if (mDirBall.prop)
      {
        os << indent << "Direction Ball Property: " << mDirBall.prop << "\n";
      }
      else
      {
        os << indent << "Direction Ball Property: (none)\n";
      }
      if (mDirBall.propSelected)
      {
        os << indent << "Selected Direction Ball Property: " << mDirBall.propSelected << "\n";
      }
      else
      {
        os << indent << "Selected Cone Property: (none)\n";
      }

      if (PlaneProperty)
      {
        os << indent << "Plane Property: " << PlaneProperty << "\n";
      }
      else
      {
        os << indent << "Plane Property: (none)\n";
      }
      if (SelectedPlaneProperty)
      {
        os << indent << "Selected Plane Property: " << SelectedPlaneProperty << "\n";
      }
      else
      {
        os << indent << "Selected Plane Property: (none)\n";
      }

      if (OutlineProperty)
      {
        os << indent << "Outline Property: " << OutlineProperty << "\n";
      }
      else
      {
        os << indent << "Outline Property: (none)\n";
      }
      if (SelectedOutlineProperty)
      {
        os << indent << "Selected Outline Property: " << SelectedOutlineProperty << "\n";
      }
      else
      {
        os << indent << "Selected Outline Property: (none)\n";
      }

      if (EdgesProperty)
      {
        os << indent << "Edges Property: " << EdgesProperty << "\n";
      }
      else
      {
        os << indent << "Edges Property: (none)\n";
      }

      os << indent << "Normal To X Axis: "
        << (NormalToXAxis ? "On" : "Off") << "\n";
      os << indent << "Normal To Y Axis: "
        << (NormalToYAxis ? "On" : "Off") << "\n";
      os << indent << "Normal To Z Axis: "
        << (NormalToZAxis ? "On" : "Off") << "\n";
      os << indent << "Lock Normal To Camera: "
        << (LockNormalToCamera ? "On" : "Off") << "\n";

      os << indent << "Widget Bounds: " << WidgetBounds[0] << ", "
        << WidgetBounds[1] << ", "
        << WidgetBounds[2] << ", "
        << WidgetBounds[3] << ", "
        << WidgetBounds[4] << ", "
        << WidgetBounds[5] << "\n";

      os << indent << "Tubing: " << (Tubing ? "On" : "Off") << "\n";
      os << indent << "Outline Translation: "
        << (OutlineTranslation ? "On" : "Off") << "\n";
      os << indent << "Outside Bounds: "
        << (OutsideBounds ? "On" : "Off") << "\n";
      os << indent << "Constrain to Widget Bounds: "
        << (ConstrainToWidgetBounds ? "On" : "Off") << "\n";
      os << indent << "Scale Enabled: "
        << (ScaleEnabled ? "On" : "Off") << "\n";
      os << indent << "Draw Plane: " << (DrawPlane ? "On" : "Off") << "\n";
      os << indent << "Bump Distance: " << BumpDistance << "\n";

      os << indent << "Representation State: ";
      switch (RepresentationState)
      {
        case enOutside:
          os << "Outside\n";
          break;
        case enMoving:
          os << "Moving\n";
          break;
        case enMovingOutline:
          os << "MovingOutline\n";
          break;
        case enMovingOrigin:
          os << "MovingOrigin\n";
          break;
        case enRotating:
          os << "Rotating\n";
          break;
        case enPushing:
          os << "Pushing\n";
          break;
        case enScaling:
          os << "Scaling\n";
          break;
        case enMovingDirectionBall:
          os << "MovingDirectionBall\n";
          break;
      }
    }

    // \brief Creates the default properties for the Objects
    void PlaneRegionRepresentation::CreateDefaultProperties()
    {
      // Normal properties
      NormalProperty = vtkProperty::New();
      NormalProperty->SetColor(1, 1, 1);
      NormalProperty->SetLineWidth(2);

      SelectedNormalProperty = vtkProperty::New();
      SelectedNormalProperty->SetColor(1, 0, 0);
      NormalProperty->SetLineWidth(2);

      // Direction Ball properties
      mDirBall.prop = make_vtk<vtkProperty>();
      mDirBall.prop->SetColor(1, 1, 1);
      mDirBall.propSelected = make_vtk<vtkProperty>();
      mDirBall.propSelected->SetColor(0, 0, 1);

      // Plane properties
      PlaneProperty = vtkProperty::New();
      PlaneProperty->SetAmbient(1.0);
      PlaneProperty->SetAmbientColor(1.0, 1.0, 1.0);
      PlaneProperty->SetOpacity(0.5);
      CutActor->SetProperty(PlaneProperty);

      SelectedPlaneProperty = vtkProperty::New();
      SelectedPlaneProperty->SetAmbient(1.0);
      SelectedPlaneProperty->SetAmbientColor(0.0, 1.0, 0.0);
      SelectedPlaneProperty->SetOpacity(0.25);

      // Outline properties
      OutlineProperty = vtkProperty::New();
      OutlineProperty->SetAmbient(1.0);
      OutlineProperty->SetAmbientColor(1.0, 1.0, 1.0);

      SelectedOutlineProperty = vtkProperty::New();
      SelectedOutlineProperty->SetAmbient(1.0);
      SelectedOutlineProperty->SetAmbientColor(0.0, 1.0, 0.0);

      // Edge property
      EdgesProperty = vtkProperty::New();
      EdgesProperty->SetAmbient(1.0);
      EdgesProperty->SetAmbientColor(1.0, 1.0, 1.0);
    }

    /**
    *
    */
    void PlaneRegionRepresentation::HighlightDirectionBall(int highlight)
    {
      if (highlight)
      {
        LineActor->SetProperty(NormalProperty);
        ConeActor->SetProperty(NormalProperty);
        LineActor2->SetProperty(NormalProperty);
        ConeActor2->SetProperty(NormalProperty);
        mDirBall.sphereActor->SetProperty(mDirBall.propSelected);
        mDirBall.linesActor->SetProperty(SelectedNormalProperty);
        SphereActor->SetProperty(NormalProperty);
      }
      else
      {
        LineActor->SetProperty(NormalProperty);
        ConeActor->SetProperty(NormalProperty);
        LineActor2->SetProperty(NormalProperty);
        ConeActor2->SetProperty(NormalProperty);
        mDirBall.sphereActor->SetProperty(mDirBall.prop);
        mDirBall.linesActor->SetProperty(SelectedNormalProperty);
        SphereActor->SetProperty(NormalProperty);
      }
    }

    // \brief Hightlights the selected Objects
    void PlaneRegionRepresentation::HighlightNormal(int highlight)
    {
      if (highlight)
      {
        LineActor->SetProperty(SelectedNormalProperty);
        ConeActor->SetProperty(SelectedNormalProperty);
        LineActor2->SetProperty(SelectedNormalProperty);
        ConeActor2->SetProperty(SelectedNormalProperty);
        mDirBall.sphereActor->SetProperty(mDirBall.propSelected);
        mDirBall.linesActor->SetProperty(SelectedNormalProperty);
        SphereActor->SetProperty(SelectedNormalProperty);
      }
      else
      {
        LineActor->SetProperty(NormalProperty);
        ConeActor->SetProperty(NormalProperty);
        LineActor2->SetProperty(NormalProperty);
        ConeActor2->SetProperty(NormalProperty);
        mDirBall.sphereActor->SetProperty(mDirBall.prop);
        mDirBall.linesActor->SetProperty(SelectedNormalProperty);
        SphereActor->SetProperty(NormalProperty);
      }
    }

    /**
    */
    void PlaneRegionRepresentation::setHandles(bool on)
    {

      mDirBall.linesActor->SetVisibility(on);
      mDirBall.sphereActor->SetVisibility(on);
      CutActor->SetVisibility(on);
      EdgesActor->SetVisibility(on);
      LineActor->SetVisibility(on);
      SphereActor->SetVisibility(on);
      ConeActor->SetVisibility(on);
      OutlineActor->SetVisibility(on);
      mArrows.getActor()->SetVisibility( ! on);
      Modified();
      BuildRepresentation();
    }
  }
}

namespace
{
  void adjustBounds(Vec3d& min, Vec3d& max)
  {
    Vec3d dist   = 0.5*(max-min);
    Vec3d center = 0.5*(min + max);
    const auto* d = dist.data();
    auto idxMin = *std::min_element(d, d+3);
    auto idxMax = *std::max_element(d, d+3);
    min[idxMin] = center[idxMin] - dist[idxMax];
    max[idxMin] = center[idxMin] + dist[idxMax];
  }
}