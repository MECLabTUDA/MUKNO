#include "private/muk.pch"
#include "private/DefaultInteraction3D.h"

#include "AppControllers.h"
#include "InteractionController.h"
#include "ProblemDefinitionController.h"
#include "VisualizationController.h"

#include "MukAppModels/AppModels.h"
#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/ProblemDefinitionModel.h"
#include "MukAppModels/VisualizationModel.h"

#include "MukCommon/IPathPlanner.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/vtk_tools.h"

#include "MukEvaluation/statistics.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/PolyDataHandler.h"
#include "MukVisualization/VisAxes.h"
#include "MukVisualization/VisCursor3D.h"
#include "MukVisualization/VisDistanceLine.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisStateRegion.h"
#include "MukVisualization/Vis3dPlane.h"

#include "MukQt/MukQRightClickMenu.h"

#include <vtkAbstractPicker.h>
#include <vtkCamera.h>
#include <vtkCoordinate.h>
#include <vtkLine.h>
#include <vtkLineSource.h>
#include <vtkObjectFactory.h>
#include <vtkPlaneSource.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>

#include <Eigen/Dense>

namespace
{
  using namespace gris::muk;

  inline bool rightKeyActivated()
  {
    return (GetKeyState(VK_RBUTTON) & 0x80) != 0;
  }

  inline bool rightKeyActivated(const char c)
  {
    return (GetKeyState(VK_RBUTTON) & 0x80) != 0;
  }

  template<typename T>
  std::shared_ptr<T> getOrCreate(const std::string& name, const VisScene& visScene, VisualizationController* pControl);

  enum class EnMouseInteraction
  {
    none,
    move,
    wheelForward,
    wheelBackward,
  };

  enum class EnInteraction
  {
    none,
    createRightClickMenu,
    waypointInteraction,
  };
}

namespace gris
{
namespace muk
{
  /**
  */
  struct DefaultInteraction3D::Impl 
  {
    Impl()
      : type(EnInteraction::none)
      , mMouseMoved(false)
      , mWaypointPicked(false)
      , pRenderer(nullptr)
    {}
    ~Impl() {}

    bool active() const { return type != EnInteraction::none; }
    bool waypointsActive() const { return mouseType != EnMouseInteraction::none; }

    // for right click movement of a waypoint
    bool mMouseMoved;
    bool mWaypointPicked;
    int  mWheelCounter;

    bool mShowDistanceLine = false;
    int  mAxesDisplayMode = 0; // 0 off, 1 axes coord system, 2 planes

    vtkRenderer* pRenderer;
    VisCursor3D* pCursor;

    EnInteraction type;
    using InteractionFunction = std::function<void(void)>;
    InteractionFunction interactionFunc;
    EnMouseInteraction  mouseType;
  };

  // ----------------------------------------------------------------------------------------

  vtkStandardNewMacro(DefaultInteraction3D);

  /**
  */
  DefaultInteraction3D::DefaultInteraction3D()
    : mp(std::make_unique<Impl>())
  {
  }
  
  /**
  */
  void DefaultInteraction3D::initialize()
  {
    mp->pCursor = mpControls->mpVisControl->requestCursor3D();
  }

  /**
  */
  void DefaultInteraction3D::OnLeftButtonDown()
  {
    if (this->Interactor->GetControlKey()) // strg pressed
    {
      this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1], 0, mpRenderer);
      double picked[3];
      this->Interactor->GetPicker()->GetPickPosition(picked);
      mp->pCursor->setPosition(Vec3d(picked));
      mpModels->pVisModel->render();
    }
    // Forward event
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }

  /** \brief activates "moving the waypoint" procedure

  searches for the closest waypoint in the loaded Waypoints of the manipulator
  if closest wp is closer than a fixed distance (2 pixels), moving is activated
  */
  void DefaultInteraction3D::OnRightButtonDown()
  {
    selectInteraction();
  }

  /**
  */
  void DefaultInteraction3D::selectInteraction()
  {
    mp->type      = EnInteraction::none;
    mp->mouseType = EnMouseInteraction::none;
    const int x = this->Interactor->GetEventPosition()[0];
    const int y = this->Interactor->GetEventPosition()[1];
    // first try to determine if a state region is picked
    if ( ! mpModels->pPlanningModel->getActivePathCollection().empty())
    {
      // plane to project on
      auto* camera = mpRenderer->GetActiveCamera();
      // 3D world point behind picked pixel
      auto c1 = make_vtk<vtkCoordinate>();
      c1->SetCoordinateSystemToViewport();
      c1->SetValue(x, y);
      const double* picked3D = c1->GetComputedWorldValue(mpRenderer);
      // line, from camera origin to picked pixel
      double position[3];
      camera->GetPosition(position);
      const auto direction = Vec3d(position[0] - picked3D[0], position[1] - picked3D[1], position[2] - picked3D[2]).normalized();
      const auto origin    = Vec3d(picked3D[0], picked3D[1], picked3D[2]);
      
      /*if (mpControls->mpProbDefControl->selectVisRegion(origin, direction))
      {
        return;
      }*/
    }
    // else
    if (mp->type == EnInteraction::none)
    {
      mp->type = EnInteraction::createRightClickMenu;
      mp->interactionFunc = [&] () { this->rightClickMenuImpl(); };
    }
  }

  ///**
  //*/
  //void DefaultInteraction3D::moveWaypointImpl()
  //{
  //  if ( ! mp->waypointsActive())
  //  {
  //    return;
  //  }
  //  auto* camera = mpRenderer->GetActiveCamera();
  //  using namespace Eigen;
  //  Vec3d tmp = mpModels->pProbDefModel->getVisRegion()->getCenter().coords;
  //  Vector3d newPoint;
  //  if (mp->mouseType == EnMouseInteraction::move)
  //  {        
  //    // plane to project on
  //    const double* normal = camera->GetDirectionOfProjection();
  //    const Vector3d n(normal[0], normal[1], normal[2]);
  //    const Vector3d e(tmp.data());
  //    Hyperplane<double, 3> plane(n, e);
  //    // 3D world point behind picked pixel
  //    auto c1 = make_vtk<vtkCoordinate>();
  //    c1->SetCoordinateSystemToViewport();
  //    c1->SetValue(this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1]); // pixel x y
  //    const double* picked3D = c1->GetComputedWorldValue(mpRenderer);
  //    // line, that gets projected onto the plane
  //    double position[3];
  //    camera->GetPosition(position);
  //    Vector3d direction(position[0] - picked3D[0], position[1] - picked3D[1], position[2] - picked3D[2]);
  //    direction.normalize();
  //    const Vector3d origin(picked3D[0], picked3D[1], picked3D[2]);
  //    ParametrizedLine<double, 3> line(origin, direction);        
  //    newPoint = line.intersectionPoint(plane);
  //  }
  //  else if (mp->mouseType == EnMouseInteraction::wheelForward || mp->mouseType == EnMouseInteraction::wheelBackward)
  //  {    
  //    const Vector3d currentPosition (tmp.data());
  //    double position[3];
  //    camera->GetPosition(position);
  //    Vector3d n  (tmp.x() - position[0], tmp.y() -position[1], tmp.z() -position[2]);
  //    n.normalize();
  //    // do not linearly increase but in relation to distance
  //    const double t = std::min(0.1 *(1 + std::pow(abs(mp->mWheelCounter), 1.5)), 1.0);
  //    if (mp->mouseType == EnMouseInteraction::wheelForward)
  //      newPoint = currentPosition + t*n;
  //    else
  //      newPoint = currentPosition - t*n;
  //  }
  //  mpModels->pProbDefModel->setPosition(Vec3d(newPoint.data()));

  //  doDistanceMeasurement(Vec3d(newPoint.data()));
  //  if (mp->mAxesDisplayMode == 1) 
  //    drawAxesLines(Vec3d(newPoint.data()));
  //  else if (mp->mAxesDisplayMode == 2)
  //    drawAxesPlanes(Vec3d(newPoint.data()));
  //  const auto p = Vec3d(newPoint.data());
  //}

  /**
  */
  void DefaultInteraction3D::rightClickMenuImpl()
  {
    if (mp->type == EnInteraction::createRightClickMenu && mp->mouseType == EnMouseInteraction::none)
    {
      auto menu = std::make_unique<MukQRightClickMenu>();
      menu->addHandle("Reposition current region", std::bind(&DefaultInteraction3D::repositionRegion, this));
      menu->addHandle("Create Sphere Region",    [&] () { emit activateInteraction("SphereRegionInteraction"); });
      menu->addHandle("Create Surface Region", [&] () { emit activateInteraction("SurfaceRegionInteraction"); });
      menu->addHandle("Create Plane Region",   [&] () { emit activateInteraction("PlaneRegionInteraction"); });
      menu->addHandle("Create Multi-Port-Region",    [&] () { emit activateInteraction("MultiPortSphereRegionInteraction"); });
      menu->addSeparator();
      menu->addHandle("Cut out new Obstacle", std::bind(&DefaultInteraction3D::cutOutObstacle, this));
      menu->addSeparator();
      menu->addHandle("Create Kappa Sphere", std::bind(&DefaultInteraction3D::createSphere, this));
      menu->addHandle("Show nearest neighbor", std::bind(&DefaultInteraction3D::showClosestPoint, this));
      menu->addHandle("Show closest point to path", std::bind(&DefaultInteraction3D::showClosestPointToPath, this));
      menu->addSeparator();
      menu->addHandle("Set Default Focus", std::bind(&DefaultInteraction3D::setDefaultFocus, this));
      menu->addSeparator();
      menu->addHandle("Toggle Distance Line", std::bind(&DefaultInteraction3D::toggleDistanceLine, this));
      menu->addHandle("Toggle Axes", std::bind(&DefaultInteraction3D::toggleAxes, this));
      menu->exec();
    }
  }

  /** \brief Activates the popup menu if no moving was performed
  */
  void DefaultInteraction3D::OnRightButtonUp()
  {
    if (mp->waypointsActive())
    {
      mp->mMouseMoved = false;
      mp->mWaypointPicked = false;
      mp->mWheelCounter = 0;
    }
    else if (mp->active())
    {
      mp->mouseType = EnMouseInteraction::none;
      mp->interactionFunc();
    }
    mp->type = EnInteraction::none;
    mp->mouseType = EnMouseInteraction::none;
  }

  /** \brief calculates the new Position of the waypoint, if activated

  moves the waypoint on the plane with the direction of projection as normal and the waypoint as point on the plane.
  see /VTK/Rendering/Testing/Cxx/otherCoordinate.cxx for example of transforming coordinates
  */
  void DefaultInteraction3D::OnMouseMove()
  {    
    mp->mouseType = EnMouseInteraction::move;
    if (rightKeyActivated() && mp->active())
    {
      if(mp->type == EnInteraction::waypointInteraction)
      {
        mp->interactionFunc();
      }
    }
    else
    {
      vtkInteractorStyleTrackballCamera::OnMouseMove();
    }
  }

  /** \brief moves the waypoint in direction of the Vector v = activeWaypoint-cameraPosition (i.e. into the screen)
  */
  void DefaultInteraction3D::OnMouseWheelForward()
  {
    mp->mouseType = EnMouseInteraction::wheelForward;
    if (rightKeyActivated())
    {
      if(mp->type == EnInteraction::waypointInteraction)
      {
        mp->interactionFunc();
      }
      ++mp->mWheelCounter;
    }
    else
    {
      vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
    }
  }

  /**
  */
  void DefaultInteraction3D::OnKeyPress()
  {
  }

  /** \brief shows Closest Point to the Cursor set by a double click in the path Analyser
  */
  void DefaultInteraction3D::OnMouseDoubleClick()
  {
    showClosestPoint();
  }

  /** \brief moves the waypoint in direction of the Vector v = cameraPosition-activeWaypoint (i.e. out of the screen)
  */
  void DefaultInteraction3D::OnMouseWheelBackward()
  {
    mp->mouseType = EnMouseInteraction::wheelBackward;
    if (rightKeyActivated())
    {
      if(mp->type == EnInteraction::waypointInteraction)
      {          
        mp->interactionFunc();
      }        
      --mp->mWheelCounter;
    }
    else
    {
      vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
    }
  }
  
  /**
  */
  void DefaultInteraction3D::repositionRegion()
  {
    //mpModels->pProbDefModel->setPosition(mp->pCursor->getPosition());
  }

  /** \brief shows a sphere with center at the selected surface point and with radius 1/kappa
  */
  void DefaultInteraction3D::createSphere()
  {
    auto sphere = make_vtk<vtkSphereSource>();
    {
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      double kappa = mpModels->pAppModel->getScene()->getPathCollection(key).getProblemDefinition()->getKappa();
      sphere->SetRadius(1 / kappa);
      auto p = mp->pCursor->getPosition();
      sphere->SetCenter(p.x(), p.y(), p.z());
      sphere->SetPhiResolution(100);
      sphere->SetThetaResolution(100);
      sphere->Update();
    }

    auto pObj = getOrCreate<VisAbstractObject>("SphereKappa", *mpModels->pVisModel->getVisScene(), mpControls->mpVisControl.get());
    pObj->setOpacity(0.75);
    pObj->setDefaultColor(Colors::Brown);
    pObj->setData(sphere->GetOutput());
    mpModels->pVisModel->render();
  }

  /** \brief computes the closest point of one of the risk structures to the selected Point and visualizes a line between them
  */
  void DefaultInteraction3D::showClosestPoint()
  {
    auto pObj = getOrCreate<VisAbstractObject>("ClosestToPoint", *mpModels->pVisModel->getVisScene(), mpControls->mpVisControl.get());
    pObj->setColors(Vec3d(1, 1, 1));
    pObj->setOpacity(1);

    auto points = make_vtk<vtkPoints>();
    auto p = mp->pCursor->getPosition();
    points->InsertNextPoint(p[0], p[1], p[2]);
    Vec3d query(p[0], p[1], p[2]);
    Vec3d nn;
    const auto& detect = *mpModels->pAppModel->getScene()->getCollisionDetector();
    if (detect.isOutOfDate())
      detect.rebuild();
    if (detect.nearestNeighbor(query, nn, std::numeric_limits<double>::max()))
    {
      points->InsertNextPoint(nn.data());
    }
    else
    {
      points->InsertNextPoint(query.data());
    }
    auto distance = (query - nn).norm();
    LOG_LINE << "query: " << query << " to " << nn << "   distance: " << (query - nn).norm();
    auto data = make_vtk<vtkPolyData>();
    data->SetPoints(points);
    PolyDataHandler::addLines(data);
    const double maxSafetyDistance = 2.5;
    const double maxDistance = 2.0;
    auto colorSpectrum = std::max(0.0, maxSafetyDistance - distance); //Values from 2(red) - 0(green)
    pObj->setColors(Vec3d(colorSpectrum/maxDistance, (maxDistance-colorSpectrum)/maxDistance, 0));
    pObj->setData(data);
    mpModels->pVisModel->render();
  }

  /** \brief computes the closest point on one of the risk structures to the first path of the active MukPath
  */
  void DefaultInteraction3D::showClosestPointToPath()
  {
    auto index = mpModels->pPlanningModel->getActivePathIdx();
    auto* pVisScene = mpModels->pVisModel->getVisScene();
    auto pColl = pVisScene->getPathCollection(mpModels->pPlanningModel->getActivePathCollection());
    if (0 == pColl->numberOfPaths())
      return;
    auto path = pColl->getMukPath(index)->asMukPath();
    if (path.getStates().empty())
      return;
    auto pDetector = mpModels->pAppModel->getScene()->getCollisionDetector();
    std::vector<double> distances = computeDistances(*pDetector, path.getStates(), path.getRadius());
    auto iter = std::min_element(distances.begin(), distances.end());
    size_t minIdx = std::distance(distances.begin(), iter);
    auto points = make_vtk<vtkPoints>();
    const Vec3d& query = path.getStates()[minIdx].coords;
    points->InsertNextPoint(query.data());
    Vec3d nn;
    if (pDetector->nearestNeighbor(query, nn, std::numeric_limits<double>::max()))
    {
      points->InsertNextPoint(nn.data());
    }
    else
    {
      points->InsertNextPoint(query.data());
    }
    LOG_LINE << "query: " << pColl->getName() << "-> from  " << query << " to " << nn << "   distance: " << *iter;
    auto data = make_vtk<vtkPolyData>();
    data->SetPoints(points);
    PolyDataHandler::addLines(data);
    auto pObj = getOrCreate<VisAbstractObject>("ClosestToPath", *pVisScene, mpControls->mpVisControl.get());
    pObj->setDefaultColor(Vec3d(1, 0, 0));
    pObj->setData(data);
    mpModels->pVisModel->render();
  }

  /**
  */
  void DefaultInteraction3D::setDefaultFocus()
  {
    auto p = mp->pCursor->getPosition();
    mpRenderer->GetActiveCamera()->SetFocalPoint(p[0], p[1], p[2]);
    mpModels->pVisModel->render();
  }

  /** \brief Calculates the distance between the @point and the nearest risk structure. 

    Prints the distance with LOG_LINE 
    Generates/Updates a VisDistanceLine that visualizes the distance. 
  */	
  void DefaultInteraction3D::doDistanceMeasurement(const Vec3d& point)
  {
    if ( ! mp->mShowDistanceLine) 
      return;
    Vec3d query(point);
    Vec3d nn;
    if ( ! mpModels->pAppModel->getScene()->getCollisionDetector()->nearestNeighbor(query, nn, std::numeric_limits<double>::max()))
    {
      LOG_LINE << "Error in nearestNeighbour calculation.  DefaultInteraction3D::doDistanceMeasurement";
    }
    double distance = (query - nn).norm();
    LOG_LINE << "distance: " << distance;

    auto distanceLine = vtkSmartPointer<vtkLineSource>::New();
    distanceLine->SetPoint1(point.x(), point.y(), point.z());
    distanceLine->SetPoint2(nn.x(), nn.y(), nn.z());
    distanceLine->Update();

    auto pObj = getOrCreate<VisDistanceLine>("distanceLine", *mpModels->pVisModel->getVisScene(), mpControls->mpVisControl.get());
    pObj->setData(distanceLine->GetOutput());
    pObj->setDistance(distance);
  }

  /** \brief Generates/Updates VisAxes which visualize the 3d coordinate system, only translated to have the @point as origin.
  */	
  void DefaultInteraction3D::drawAxesLines(const Vec3d& point)
  {
    int length = 10;

    // calculate the endpoints of the lines 
    auto pts = vtkSmartPointer<vtkPoints>::New();
    pts->InsertNextPoint(point.x() - length, point.y(), point.z());
    pts->InsertNextPoint(point.x() + length, point.y(), point.z());
    pts->InsertNextPoint(point.x(), point.y() - length, point.z());
    pts->InsertNextPoint(point.x(), point.y() + length, point.z());
    pts->InsertNextPoint(point.x(), point.y(), point.z() + length);
    pts->InsertNextPoint(point.x(), point.y(), point.z() - length);

    // connect the points
    auto xAxisLine = vtkSmartPointer<vtkLine>::New();
    xAxisLine->GetPointIds()->SetId(0, 0);
    xAxisLine->GetPointIds()->SetId(1, 1);
    auto yAxisLine = vtkSmartPointer<vtkLine>::New();
    yAxisLine->GetPointIds()->SetId(0, 2);
    yAxisLine->GetPointIds()->SetId(1, 3);
    auto zAxisLine = vtkSmartPointer<vtkLine>::New();
    zAxisLine->GetPointIds()->SetId(0, 4);
    zAxisLine->GetPointIds()->SetId(1, 5);

    auto lines = vtkSmartPointer<vtkCellArray>::New();
    lines->InsertNextCell(xAxisLine);
    lines->InsertNextCell(yAxisLine);
    lines->InsertNextCell(zAxisLine);

    auto pdata = vtkSmartPointer<vtkPolyData>::New();
    pdata->SetPoints(pts);
    pdata->SetLines(lines);

    auto pObj = getOrCreate<VisDistanceLine>("3dAxes", *mpModels->pVisModel->getVisScene(), mpControls->mpVisControl.get());
    pObj->setData(pdata);
  }

  /** \brief Generates/Updates Vis3dPlanes which visualize the 3 planes of the coordinate system translated to have @point as origin.

  The zPlane has default visibility of true		
  x and y Plane have default visibility of false
  */	
  void DefaultInteraction3D::drawAxesPlanes(const Vec3d& point)
  {
    int length = 10;

    auto zPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
    zPlaneSource->SetOrigin(point.x() - length, point.y() - length, point.z());
    zPlaneSource->SetPoint1(point.x() + length, point.y() - length, point.z());
    zPlaneSource->SetPoint2(point.x() - length, point.y() + length, point.z());
    zPlaneSource->Update();

    auto yPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
    yPlaneSource->SetOrigin(point.x() - length, point.y(), point.z() - length);
    yPlaneSource->SetPoint1(point.x() + length, point.y(), point.z() - length);
    yPlaneSource->SetPoint2(point.x() - length, point.y(), point.z() + length);
    yPlaneSource->Update();

    auto xPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
    xPlaneSource->SetOrigin(point.x(), point.y() - length, point.z() - length);
    xPlaneSource->SetPoint1(point.x(), point.y() - length, point.z() + length);
    xPlaneSource->SetPoint2(point.x(), point.y() + length, point.z() - length);
    xPlaneSource->Update();

    // dummy texture
    /* auto imageSource = vtkSmartPointer<vtkImageCanvasSource2D>::New();
    imageSource->SetScalarTypeToUnsignedChar();
    imageSource->SetExtent(0, 20, 0, 20, 0, 0);
    imageSource->SetNumberOfScalarComponents(3);
    imageSource->SetDrawColor(127, 255, 100);
    imageSource->FillBox(0, 20, 0, 20);
    imageSource->SetDrawColor(20, 20, 20);
    imageSource->DrawSegment(0, 0, 19, 19);
    imageSource->DrawSegment(19, 0, 0, 19);
    imageSource->Update();

    auto lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetTableRange(0, 255);
    lut->SetHueRange(0.0, 0.0);
    lut->SetSaturationRange(0.0, 0.0);
    lut->SetValueRange(0.0, 1.0);
    lut->Build();

    auto texture = vtkSmartPointer<vtkTexture>::New();
    texture->SetLookupTable(lut);
    texture->SetInputData(imageSource->GetOutput());*/


    {
      auto pObj = getOrCreate<Vis3dPlane>("zPlane", *mpModels->pVisModel->getVisScene(), mpControls->mpVisControl.get());
      pObj->setData(zPlaneSource->GetOutput());
      //obj->setTexture(texture); 
    }
    {
      auto pObj = getOrCreate<Vis3dPlane>("xPlane", *mpModels->pVisModel->getVisScene(), mpControls->mpVisControl.get());
      pObj->setData(xPlaneSource->GetOutput());
    }
    {
      auto pObj = getOrCreate<Vis3dPlane>("yPlane", *mpModels->pVisModel->getVisScene(), mpControls->mpVisControl.get());
      pObj->setData(yPlaneSource->GetOutput());
    }
  }

  /** \brief Toggles between showing a distance line between a waypoint and the nearest structure
  */
  void DefaultInteraction3D::toggleDistanceLine()
  {
    mp->mShowDistanceLine = ! mp->mShowDistanceLine;
    LOG_LINE << "Distance line drawing is now " << mp->mShowDistanceLine;

    // if neccessary delete the existing line
    if (!mp->mShowDistanceLine)
    {
      auto keys = mpModels->pVisModel->getAbstractObjectKeys();
      if (std::find(keys.begin(), keys.end(), "distanceLine") != keys.end())
      {
        mpControls->mpVisControl->deleteAbstractObject("distanceLine");
      }
    }
  }

  /** \brief Toggles between showing various helpers to display the axes of the coordinate system.

  0: No helpers
  1: 3d axes 
  2: 3d planes 
  */
  void DefaultInteraction3D::toggleAxes()
  {
    mp->mAxesDisplayMode = (mp->mAxesDisplayMode + 1) % 3;

    // delete the objects from the old mode
    if (mp->mAxesDisplayMode == 0)
    {
      LOG_LINE << "Axes drawing is: nothing";
      auto keys = mpModels->pVisModel->getAbstractObjectKeys();
      if (std::find(keys.begin(), keys.end(), "zPlane") != keys.end())
      {
        mpControls->mpVisControl->deleteAbstractObject("zPlane");
      }
      if (std::find(keys.begin(), keys.end(), "xPlane") != keys.end())
      {
        mpControls->mpVisControl->deleteAbstractObject("xPlane");
      }
      if (std::find(keys.begin(), keys.end(), "yPlane") != keys.end())
      {
        mpControls->mpVisControl->deleteAbstractObject("yPlane");
      }
    }
    else if (mp->mAxesDisplayMode == 2)
    {
      LOG_LINE << "Axes drawing is: planes";
      auto keys = mpModels->pVisModel->getAbstractObjectKeys();
      if (std::find(keys.begin(), keys.end(), "3dAxes") != keys.end())
      {
        mpControls->mpVisControl->deleteAbstractObject("3dAxes");
      }
    }
    else
    {
      LOG_LINE << "Axes drawing is: 3d axes";
    }
  }

  /**
  */
  void DefaultInteraction3D::activateInteraction(const std::string& key)
  {
    mpControls->mpInteract->setInteraction(key);
  }
    
  /**
  */
  void DefaultInteraction3D::cutOutObstacle()
  {
    mpControls->mpInteract->setInteraction("cutOutObstacle");
  }
}
}

namespace
{
  /**
  */
  template<typename T>
  std::shared_ptr<T> getOrCreate(const std::string& name, const VisScene& visScene, VisualizationController* pControl)
  {
    auto keys = visScene.getAbstractObjectKeys();
    if (std::none_of(keys.begin(), keys.end(), [&] (const auto& str) { return name == str; }))
    {
      auto pObj = std::make_shared<T>(name);
      pControl->addAbstractObject(pObj);
    }
    return std::dynamic_pointer_cast<T>(visScene.getObject(name));
  }
}