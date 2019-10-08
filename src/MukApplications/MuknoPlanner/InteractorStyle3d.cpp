#include "private/muk.pch"
#include "private/InteractorStyle3D.h"
#include "private/RegionMarker.h"

#include "AppLogics.h"
#include "ApplicationLogic.h"
#include "PlanningLogic.h"
#include "WaypointManipulator.h"
#include "VisualizationLogic.h"

#include "MukCommon/PathCollection.h"
#include "MukEvaluation/statistics.h"
#include "MukVisualization/PolyDataHandler.h"
#include "MukVisualization/VisWaypoints.h"
#include "MukVisualization/VisAbstractObject.h"
#include "MukVisualization/VisPathCollection.h"

#include "MukQt/MukQRightClickMenu.h"

#include <vtkObjectFactory.h>
#include <vtkPoints.h>
#include <vtkProperty.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPointData.h>
#include <vtkSphereSource.h>

#include <vtkAbstractPicker.h>
#include <vtkObjectFactory.h>
#include <vtkAxisActor.h>
#include <vtkCamera.h>
#include <vtkCoordinate.h>

#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkInteractorObserver.h>

#include <Eigen/Dense>

#include <boost/format.hpp>

#include <omp.h>
#include <windows.h>

#include "MukQt/VtkWindow.h"
#include "VisDistanceLine.h"
#include "VisAxes.h"
#include "Vis3dPlane.h"

#include "vtkImageCanvasSource2D.h"

#include "CtWindow.h"
#include <vtkLineSource.h>
#include <vtkPlaneSource.h>
#include "vtkLookupTable.h"
#include "vtkLine.h"
#include "vtkCellArray.h"
#include <QTabWidget>
#include <QWidget>


#ifdef min
#undef min
#endif

namespace
{
	inline bool rightKeyActivated()
	{
		return (GetKeyState(VK_RBUTTON) & 0x80) != 0;
	}

	inline bool rightKeyActivated(const char c)
	{
		return (GetKeyState(VK_RBUTTON) & 0x80) != 0;
	}
}

namespace gris
{
	namespace muk
	{
		vtkStandardNewMacro(InteractorStyle3D);

		/**
		*/
		InteractorStyle3D::InteractorStyle3D()
			: mMouseMoved(false)
			, mWaypointPicked(false)
			, mpRegionMarker(std::make_unique<RegionMarker>(this))
		{
			
		}

		/** \brief Also initializes member variables used for the selectable pixel
		*/
		void InteractorStyle3D::setRenderer(vtkRenderer* renderer)
		{
			mRenderer = renderer;

			mPointSelectedPixel = NewVtk(vtkPolyData);
			DefVtk(vtkPoints, points);
			points->InsertNextPoint(0, 0, 0);
			mPointSelectedPixel->SetPoints(points);
			PolyDataHandler::addVertices(mPointSelectedPixel.Get());
			mMapperSelectedPixel = NewVtk(vtkPolyDataMapper);
			mMapperSelectedPixel->SetInputData(mPointSelectedPixel);
			mActorSelectedPixel = NewVtk(vtkActor);
			mActorSelectedPixel->SetMapper(mMapperSelectedPixel);
			mActorSelectedPixel->GetProperty()->SetColor(gris::muk::Colors::Green);
			mActorSelectedPixel->GetProperty()->SetPointSize(5.0);
			mRenderer->AddActor(mActorSelectedPixel);

			mpRegionMarker->setRenderer(mRenderer);
		}

		/** \brief Sets the selectable point if STRG is pressed
		*/
		void InteractorStyle3D::OnLeftButtonDown()
		{
			if (this->Interactor->GetControlKey()) // strg pressed
			{
				this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1], 0, mRenderer);
				double picked[3];
				this->Interactor->GetPicker()->GetPickPosition(picked);

				mPointSelectedPixel->GetPoints()->SetPoint(0, picked);
				PolyDataHandler::addVertices(mPointSelectedPixel.Get());
				mMapperSelectedPixel->SetInputData(mPointSelectedPixel);
				mMapperSelectedPixel->Update();
			}
			// Forward event
			vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
		}

		/** \brief activates "moving the waypoint" procedure

		  searches for the closest waypoint in the loaded Waypoints of the manipulator
		  if closest wp is closer than a fixed distance (2 pixels), moving is activated
		*/
		void InteractorStyle3D::OnRightButtonDown()
		{
			const int x = this->Interactor->GetEventPosition()[0];
			const int y = this->Interactor->GetEventPosition()[1];
			auto pObj = mpLogics->pWpManipulator->getVisualization();
			if (nullptr == pObj.get())
				return;
			auto data = pObj->getData();
			if (nullptr == data.Get())
				return;
			const size_t N = data->GetNumberOfPoints();
			auto c1 = make_vtk<vtkCoordinate>();
			const int Max_Manhattan_Distance = 2;
			for (size_t i(0); i < N; ++i)
			{
				double d[3];
				data->GetPoint(i, d);
				c1->SetCoordinateSystemToWorld();
				c1->SetValue(d);
				int* j = c1->GetComputedViewportValue(mRenderer);
				if (abs(j[0] - x) <= Max_Manhattan_Distance
					&& abs(j[1] - y) <= Max_Manhattan_Distance)
				{
					mWaypointPicked = true;
					wheelCounter = 0;
					mpPickedWaypoint = Vec3d(d[0], d[1], d[2]);
					mpLogics->pWpManipulator->loadWaypoint(i);
					break;
				}
			}
		}

		/** \brief Activates the popup menu if no moving was performed
		*/
		void InteractorStyle3D::OnRightButtonUp()
		{
			if (mMouseMoved)
			{
				mMouseMoved = false;
				mWaypointPicked = false;
				if (mpRegionMarker->isActive())
				{
					mpRegionMarker->processPoints();
				}
			}
			else
			{
        auto menu = std::make_unique<MukQRightClickMenu>();
		//menu->addHandle("Eliminate 50%",   std::bind(&InteractorStyle3D::eliminate50, this));
				menu->addHandle("Set as Start Point", std::bind(&InteractorStyle3D::setStartPoint, this));
				menu->addHandle("Set as Goal Point", std::bind(&InteractorStyle3D::setGoalPoint, this));
				menu->addHandle("Set as Start Region", std::bind(&InteractorStyle3D::setStartRegion, this));
				menu->addHandle("Set as Goal Region", std::bind(&InteractorStyle3D::setGoalRegion, this));
				menu->addHandle("Set as Active Waypoint", std::bind(&InteractorStyle3D::setWaypoint, this));
				menu->addHandle("Create Kappa Sphere", std::bind(&InteractorStyle3D::createSphere, this));
				menu->addHandle("Show nearest neighbor", std::bind(&InteractorStyle3D::showClosestPoint, this));
				menu->addHandle("Show closest point to path", std::bind(&InteractorStyle3D::showClosestPointToPath, this));
				menu->addSeparator();
				menu->addHandle("Set Default Focus", std::bind(&InteractorStyle3D::setDefaultFocus, this));
				menu->addSeparator();
				menu->addHandle("Toggle Distance Line", std::bind(&InteractorStyle3D::toggleDistanceLine, this));
				menu->addHandle("Toggle Axes", std::bind(&InteractorStyle3D::toggleAxes, this));
				menu->exec();
			}
		}

		/** \brief calculates the new Position of the waypoint, if activated

		  moves the waypoint on the plane with the direction of projection as normal and the waypoint as point on the plane.
		  see /VTK/Rendering/Testing/Cxx/otherCoordinate.cxx for example of transforming coordinates
		*/
		void InteractorStyle3D::OnMouseMove()
		{
			if (rightKeyActivated())
			{
				mMouseMoved = true;
				if (mWaypointPicked)
				{
					auto* camera = this->mRenderer->GetActiveCamera();
					// plane to project on
					using namespace Eigen;
					const double* normal = camera->GetDirectionOfProjection();
					const Vector3d n(normal[0], normal[1], normal[2]);
					double tmp[3];
					mpLogics->pWpManipulator->getVisualization()->getData()->GetPoint(mpLogics->pWpManipulator->getIndex(), tmp);
					const Vector3d e(tmp[0], tmp[1], tmp[2]);
					Hyperplane<double, 3> plane(n, e);

					// 3D world point behind picked pixel
					auto c1 = make_vtk<vtkCoordinate>();
					c1->SetCoordinateSystemToViewport();
					c1->SetValue(this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1]); // pixel x y
					const double* picked3D = c1->GetComputedWorldValue(mRenderer);

					// line, that gets projected onto the plane
					double position[3];
					camera->GetPosition(position);
					const Vector3d direction(position[0] - picked3D[0], position[1] - picked3D[1], position[2] - picked3D[2]);
					const Vector3d origin(picked3D[0], picked3D[1], picked3D[2]);
					ParametrizedLine<double, 3> line(origin, direction);

					// compute result and set point
					const Vector3d newPoint = line.intersectionPoint(plane);
					mpLogics->pWpManipulator->setPosition(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));


					doDistanceMeasurement(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));

					// navigation help display 
					if (mAxesDisplayMode == 1) drawAxesLines(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));
					else if (mAxesDisplayMode == 2) drawAxesPlanes(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));

					mpLogics->pVisLogic->render();

				}
				if (mpRegionMarker->isActive())
				{
					//mpRegionMarker->addPolygonPoint(Vec2d(this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1]));
					/*this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1], 0, mRenderer);
					double picked[3];
					this->Interactor->GetPicker()->GetPickPosition(picked);      */
					//mpRegionMarker->addPoint(Vec3d(picked));
					mpRegionMarker->addPoint(Vec2d(this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1]));
					//mRenderer->Render();
				}
			}
			else
			{
				vtkInteractorStyleTrackballCamera::OnMouseMove();
			}
		}

		/** \brief moves the waypoint in direction of the Vector v = activeWaypoint-cameraPosition (i.e. into the screen)
		*/
		void InteractorStyle3D::OnMouseWheelForward()
		{
			if (rightKeyActivated())
			{
				mMouseMoved = true;
				if (mWaypointPicked)
				{
					auto* camera = this->mRenderer->GetActiveCamera();
					// plane to project on
					using namespace Eigen;
					double tmp[3];
					mpLogics->pWpManipulator->getVisualization()->getData()->GetPoint(mpLogics->pWpManipulator->getIndex(), tmp);
					const Vector3d currentPosition(tmp[0], tmp[1], tmp[2]);
					double position[3];
					camera->GetPosition(position);
					Vector3d n(tmp[0] - position[0], tmp[1] - position[1], tmp[2] - position[2]);
					n.normalize();
					// do not linearly increase but in relation to distance
					const double t = std::min(0.1 *(1 + std::pow(abs(wheelCounter), 1.5)), 1.0);
					auto const newPoint = currentPosition + t*n;
					mpLogics->pWpManipulator->setPosition(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));
					++wheelCounter;

					doDistanceMeasurement(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));
					if (mAxesDisplayMode == 1) drawAxesLines(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));
					else if (mAxesDisplayMode == 2) drawAxesPlanes(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));

					mpLogics->pVisLogic->render();
				}
			}
			else
			{
				vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
			}
		}

		/**
		*/
		void InteractorStyle3D::OnKeyPress()
		{
			mpRegionMarker->setType(Interactor->GetKeyCode());
		}

		/** \brief moves the waypoint in direction of the Vector v = cameraPosition-activeWaypoint (i.e. out of the screen)
		*/
		void InteractorStyle3D::OnMouseWheelBackward()
		{
			if (rightKeyActivated())
			{
				mMouseMoved = true;
				if (mWaypointPicked)
				{
					auto* camera = this->mRenderer->GetActiveCamera();
					// plane to project on
					using namespace Eigen;
					double tmp[3];
					mpLogics->pWpManipulator->getVisualization()->getData()->GetPoint(mpLogics->pWpManipulator->getIndex(), tmp);
					const Vector3d currentPosition(tmp[0], tmp[1], tmp[2]);
					double position[3];
					camera->GetPosition(position);
					Vector3d n(tmp[0] - position[0], tmp[1] - position[1], tmp[2] - position[2]);
					n.normalize();
					// do not linearly increase but in relation to wheel movement
					const double t = std::min(0.1 *(1 + std::pow(abs(wheelCounter), 1.5)), 1.0);
					auto const newPoint = currentPosition - t*n;
					mpLogics->pWpManipulator->setPosition(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));
					--wheelCounter;

					doDistanceMeasurement(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));

					// navigation help display 
					if (mAxesDisplayMode == 1) drawAxesLines(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));
					else if (mAxesDisplayMode == 2) drawAxesPlanes(Vec3d(newPoint.x(), newPoint.y(), newPoint.z()));

					mpLogics->pVisLogic->render();
				}
			}
			else
			{
				vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
			}
		}

		/**
		*/
		Vec3d InteractorStyle3D::getSelectedPoint()
		{
			double p[3];
			mPointSelectedPixel->GetPoint(0, p);
			return Vec3d(p[0], p[1], p[2]);
		}

		/**
		*/
		void InteractorStyle3D::setStartRegion()
		{
			auto region = mpRegionMarker->getRegion();
			mpLogics->pPlanningLogic->setStartRegion(region);
		}

		/**
		*/
		void InteractorStyle3D::setGoalRegion()
		{
			std::vector<MukState> tmp;
			LOG_LINE << getSelectedPoint();
			tmp.push_back(MukState(getSelectedPoint(), Vec3d(1, 0, 0)));
			mpLogics->pPlanningLogic->setGoalRegion(tmp);
		}

	//-----------------------------
    /** TEST Funktion von Habib
	*	Eliminate 50% of the paths by distances to risk structures
		*/

	/*void InteractorStyle3D::eliminate50()
	{
		LOG_LINE << "-->>Eliminatation!<<--";

		auto pScene = mpLogics->pAppLogic->getScene();
		const auto& key = mpLogics->pPlanningLogic->getActivePathCollection();
		if (key.empty())
			return;

		auto& coll = pScene->getPathCollection(key);
		const size_t N = coll.getPaths().size();
		if (N == 0)
			return;

		std::vector<double> minDist;

		//get all min. distances
		for (size_t i = 0; i < N; ++i) {
			auto pObj = mpLogics->pVisLogic->getVisScene()->getPathCollection(key)->getMukPath(i); // take the data from the visualization!
			MukPath path = pObj->asMukPath();
			auto distances = computeDistances(*pScene->getCollisionDetector(), path.getPath(), path.getRadius());
			auto iter = *min_element(distances.begin(), distances.end());
			minDist.push_back((double) iter);
			//LOG_LINE << ">> minDist " << i << ":  " << minDist[i];
		}	
		
		std::sort(minDist.begin(), minDist.end()); //sort min. distances

		//LOG_LINE << ">>> sort minimum Distances";
		//for (int k=0; k < minDist.size(); ++k) {
		//	LOG_LINE << ">> minDist " << k << ":  " << minDist[k];
		//}

		//determin the middle value
		double limit;
		if (N%2 == 0)
			limit = minDist[N / 2];
		else
			limit = minDist[std::ceil(N / 2)];
		LOG_LINE << ">> Limit: " << limit;

		//delete all paths wit min. distance above limit
		std::vector<size_t> delete_idx;
		for (size_t x = 0; x < N; ++x) {
			auto pObj = mpLogics->pVisLogic->getVisScene()->getPathCollection(key)->getMukPath(x); // take the data from the visualization!
			MukPath path = pObj->asMukPath();
			auto distances = computeDistances(*pScene->getCollisionDetector(), path.getPath(), path.getRadius());
			auto iter = *min_element(distances.begin(), distances.end());

			if ((double)iter < limit)	delete_idx.push_back(x);
		}

		for (std::vector<size_t>::reverse_iterator it = delete_idx.rbegin(); it != delete_idx.rend(); ++it)
			mpLogics->pVisLogic->deletePath(key, *it);

		//size_t n = N-1;
		//while (n != 0) {
		//	auto pObj = mpLogics->pVisLogic->getVisScene()->getPathCollection(key)->getMukPath(n); // take the data from the visualization!
		//	MukPath path = pObj->asMukPath();
		//	auto distances = computeDistances(*pScene->getCollisionDetector(), path.getPath(), path.getRadius());
		//	auto iter = *min_element(distances.begin(), distances.end());

		//	if ((double)iter < limit) mpLogics->pVisLogic->deletePath(key, n);
		//	n--;
		//}
		
		mRenderer->Render();
	}*/

	//-----------------------------
		void InteractorStyle3D::setStartPoint()
		{
			const auto& key = mpLogics->pPlanningLogic->getActivePathCollection();
			if (key.empty())
				return;
			const int oldIdx = mpLogics->pWpManipulator->getIndex();
			mpLogics->pWpManipulator->loadWaypoint(0);
			mpLogics->pWpManipulator->setPosition(this->getSelectedPoint());
			mpLogics->pWpManipulator->loadWaypoint(oldIdx);
		}

		void InteractorStyle3D::setWaypoint()
		{
			const auto& key = mpLogics->pPlanningLogic->getActivePathCollection();
			if (key.empty())
				return;
			mpLogics->pWpManipulator->setPosition(this->getSelectedPoint());
		}

		void InteractorStyle3D::setGoalPoint()
		{
			const auto& key = mpLogics->pPlanningLogic->getActivePathCollection();
			if (key.empty())
				return;
			const int oldIdx = mpLogics->pWpManipulator->getIndex();
			const int newIdx = mpLogics->pWpManipulator->getVisualization()->getData()->GetNumberOfPoints() - 1;
			mpLogics->pWpManipulator->loadWaypoint(newIdx);
			mpLogics->pWpManipulator->setPosition(this->getSelectedPoint());
			mpLogics->pWpManipulator->loadWaypoint(oldIdx);
		}

		/** \brief shows a sphere with center at the selected surface point and with radius 1/kappa
		*/
		void InteractorStyle3D::createSphere()
		{
			DefVtk(vtkSphereSource, sphere);
			LOG_LINE << "radius " << mpLogics->pAppLogic->getScene()->getPlanner()->getKappa();
			sphere->SetRadius(1 / mpLogics->pAppLogic->getScene()->getPlanner()->getKappa());
			const Vec3d p = this->getSelectedPoint();
			sphere->SetCenter(p.x, p.y, p.z);
			sphere->SetPhiResolution(360);
			sphere->SetThetaResolution(360);
			sphere->Update();
			const std::string key = "SphereKappa";
			auto pObj = std::make_shared<VisAbstractObject>(key);
			pObj->setOpacity(0.75);
			pObj->setDefaultColor(Colors::Brown);
			pObj->setData(sphere->GetOutput());
			mpLogics->pVisLogic->addAbstractObject(pObj);
		}

		/** \brief computes the closest point of one of the risk structures to the selected Point and visualizes a line between them
		*/
		void InteractorStyle3D::showClosestPoint()
		{
			auto pObj = std::make_shared<VisAbstractObject>("ClosestToPoint");
			pObj->setDefaultColor(Vec3d(1, 0, 0));

			DefVtk(vtkPoints, points);
			double p[3];
			mPointSelectedPixel->GetPoint(0, p);
			points->InsertNextPoint(p[0], p[1], p[2]);
			Vec3d query(p[0], p[1], p[2]);
			Vec3d nn;
			if (mpLogics->pAppLogic->getScene()->getCollisionDetector()->nearestNeighbor(query, nn, std::numeric_limits<double>::max()))
			{
				points->InsertNextPoint(nn.x, nn.y, nn.z);
			}
			else
			{
				points->InsertNextPoint(query.x, query.y, query.z);
			}
			LOG_LINE << "query: " << query << " to " << nn << "   distance: " << (query - nn).norm();
			DefVtk(vtkPolyData, data);
			data->SetPoints(points);
			PolyDataHandler::addLines(data);
			pObj->setData(data);
			mpLogics->pVisLogic->addAbstractObject(pObj);
		}

		/** \brief computes the closest point on one of the risk structures to the first path of the active MukPath
		*/
		void InteractorStyle3D::showClosestPointToPath()
		{
			auto pScene = mpLogics->pAppLogic->getScene();
			auto& coll = mpLogics->pAppLogic->getScene()->getPathCollection(mpLogics->pPlanningLogic->getActivePathCollection());
			if (coll.getPaths().empty())
				return;
			auto& path = coll.getPaths().front();
			if (path.getPath().empty())
				return;
			const double radius = coll.getRadius();
			std::vector<double> distances = computeDistances(*pScene->getCollisionDetector(), path.getPath(), radius);
			auto iter = std::min_element(distances.begin(), distances.end());
			size_t minIdx = std::distance(distances.begin(), iter);
      auto points = make_vtk<vtkPoints>();
			const Vec3d& query = path.getPath()[minIdx].coords;
			points->InsertNextPoint(query.x, query.y, query.z);
			Vec3d nn;
			if (mpLogics->pAppLogic->getScene()->getCollisionDetector()->nearestNeighbor(query, nn, std::numeric_limits<double>::max()))
			{
				points->InsertNextPoint(nn.x, nn.y, nn.z);
			}
			else
			{
				points->InsertNextPoint(query.x, query.y, query.z);
			}
			LOG_LINE << "query: " << coll.getName() << "-> from  " << query << " to " << nn << "   distance: " << *iter;
			DefVtk(vtkPolyData, data);
			data->SetPoints(points);
			PolyDataHandler::addLines(data);
			auto pObj = std::make_shared<VisAbstractObject>("ClosestToPath");
			pObj->setDefaultColor(Vec3d(1, 0, 0));
			pObj->setData(data);
			mpLogics->pVisLogic->addAbstractObject(pObj);
		}

		/**
		*/
		void InteractorStyle3D::setDefaultFocus()
		{
			double p[3];
			mPointSelectedPixel->GetPoint(0, p);
			mRenderer->GetActiveCamera()->SetFocalPoint(p[0], p[1], p[2]);
			emit mpLogics->pAppLogic->activeCameraChanged();
		}

		
		/** \brief Calculates the distance between the @point and the nearest risk structure. 
		
		Prints the distance with LOG_LINE 
		Generates/Updates a VisDistanceLine that visualizes the distance. 
		*/	
		void InteractorStyle3D::doDistanceMeasurement(const Vec3d& point)
		{
			if (!mShowDistanceLine) return;
			Vec3d query(point.x, point.y, point.z);
			Vec3d nn;
			if (!mpLogics->pAppLogic->getScene()->getCollisionDetector()->nearestNeighbor(query, nn, std::numeric_limits<double>::max()))
			{
				LOG_LINE << "Error in nearestNeighbour calculation.  InteractorStyle3D::doDistanceMeasurement";
			}
			double distance = (query - nn).norm();
			LOG_LINE << "distance: " << distance;

			auto distanceLine = vtkSmartPointer<vtkLineSource>::New();
			distanceLine->SetPoint1(point.x, point.y, point.z);
			distanceLine->SetPoint2(nn.x, nn.y, nn.z);
			distanceLine->Update();

			auto keys = mpLogics->pVisLogic->getAbstractObjectKeys();
			if (std::find(keys.begin(), keys.end(), "distanceLine") != keys.end())
			{
				std::shared_ptr<VisDistanceLine> obj = std::dynamic_pointer_cast<VisDistanceLine>(mpLogics->pVisLogic->getVisScene()->getObject("distanceLine"));
				obj->setDistance(distance);
				obj->setData(distanceLine->GetOutput());
			}
			else
			{
				auto pObj = std::make_shared<VisDistanceLine>("distanceLine");
				pObj->setDefaultColor(Vec3d(1, 0, 0.5));
				pObj->setData(distanceLine->GetOutput());
				pObj->setDistance(distance);
				pObj->setWarningColor(Vec3d(1.0, 1.0, 0.0));
				pObj->setWarningDistance(1.0);
				mpLogics->pVisLogic->addAbstractObject(pObj);
			}
		}

		/** \brief Generates/Updates VisAxes which visualize the 3d coordinate system, only translated to have the @point as origin.
		*/	
		void InteractorStyle3D::drawAxesLines(const Vec3d& point)
		{
			int length = 10;

			// calculate the endpoints of the lines 
			auto pts = vtkSmartPointer<vtkPoints>::New();
			pts->InsertNextPoint(point.x - length, point.y, point.z);
			pts->InsertNextPoint(point.x + length, point.y, point.z);
			pts->InsertNextPoint(point.x, point.y - length, point.z);
			pts->InsertNextPoint(point.x, point.y + length, point.z);
			pts->InsertNextPoint(point.x, point.y, point.z - length);
			pts->InsertNextPoint(point.x, point.y, point.z + length);

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

			auto keys = mpLogics->pVisLogic->getAbstractObjectKeys();
			if (std::find(keys.begin(), keys.end(), "3dAxes") != keys.end())
			{
				std::shared_ptr<VisAxes> obj = std::dynamic_pointer_cast<VisAxes>(mpLogics->pVisLogic->getVisScene()->getObject("3dAxes"));
				obj->setData(pdata);
			}
			else
			{
				auto pObj = std::make_shared<VisAxes>("3dAxes");
				pObj->setDefaultColor(Vec3d(0.82, 0.82, 0.82));
				pObj->setOpacity(1.0);
				pObj->setData(pdata);
				pObj->setLineStipplePattern(0xaaaa);
				mpLogics->pVisLogic->addAbstractObject(pObj);
			}
		}

		/** \brief Generates/Updates Vis3dPlanes which visualize the 3 planes of the coordinate system translated to have @point as origin.
		
		The zPlane has default visibility of true		
		x and y Plane have default visibility of false
		*/	
		void InteractorStyle3D::drawAxesPlanes(const Vec3d& point)
		{
			int length = 10;

			auto zPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
			zPlaneSource->SetOrigin(point.x - length, point.y - length, point.z);
			zPlaneSource->SetPoint1(point.x + length, point.y - length, point.z);
			zPlaneSource->SetPoint2(point.x - length, point.y + length, point.z);
			zPlaneSource->Update();

			auto yPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
			yPlaneSource->SetOrigin(point.x - length, point.y, point.z - length);
			yPlaneSource->SetPoint1(point.x + length, point.y, point.z - length);
			yPlaneSource->SetPoint2(point.x - length, point.y, point.z + length);
			yPlaneSource->Update();

			auto xPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
			xPlaneSource->SetOrigin(point.x, point.y - length, point.z - length);
			xPlaneSource->SetPoint1(point.x, point.y - length, point.z + length);
			xPlaneSource->SetPoint2(point.x, point.y + length, point.z - length);
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
			

			auto keys = mpLogics->pVisLogic->getAbstractObjectKeys();
			
			//zPlane}
			if (std::find(keys.begin(), keys.end(), "zPlane") != keys.end())
			{
				std::shared_ptr<Vis3dPlane> obj = std::dynamic_pointer_cast<Vis3dPlane>(mpLogics->pVisLogic->getVisScene()->getObject("zPlane"));
				obj->setData(zPlaneSource->GetOutput());
				//obj->setTexture(texture); 
			}
			else
			{
				auto pObj = std::make_shared<Vis3dPlane>("zPlane");
				pObj->setDefaultColor(Vec3d(0.82, 0.82, 0.82));
				pObj->setOpacity(0.5);
				pObj->setData(zPlaneSource->GetOutput());
				//pObj->setTexture(texture);
				mpLogics->pVisLogic->addAbstractObject(pObj);
			}


			// xPlane
			if (std::find(keys.begin(), keys.end(), "xPlane") != keys.end())
			{
				std::shared_ptr<VisAbstractObject> obj = mpLogics->pVisLogic->getVisScene()->getObject("xPlane");
				obj->setData(xPlaneSource->GetOutput());
			}
			else
			{
				auto pObj = std::make_shared<VisAbstractObject>("xPlane");
				pObj->setDefaultColor(Vec3d(0.82, 0.82, 0.82));
				pObj->setOpacity(0.5);
				pObj->setVisibility(false);
				pObj->setData(xPlaneSource->GetOutput());
				mpLogics->pVisLogic->addAbstractObject(pObj);
			}
			// yPlane
			if (std::find(keys.begin(), keys.end(), "yPlane") != keys.end())
			{
				std::shared_ptr<VisAbstractObject> obj = mpLogics->pVisLogic->getVisScene()->getObject("yPlane");
				obj->setData(yPlaneSource->GetOutput());
			}
			else
			{
				auto pObj = std::make_shared<VisAbstractObject>("yPlane");
				pObj->setDefaultColor(Vec3d(0.82, 0.82, 0.82));
				pObj->setOpacity(0.5);
				pObj->setVisibility(false);
				pObj->setData(yPlaneSource->GetOutput());
				mpLogics->pVisLogic->addAbstractObject(pObj);
			}

		}


		/** \brief Toggles between showing a distance line between a waypoint and the nearest structure
		*/
		void InteractorStyle3D::toggleDistanceLine()
		{
			mShowDistanceLine = !mShowDistanceLine;
			LOG_LINE << "Distance line drawing is now " << mShowDistanceLine;

			// if neccessary delete the existing line
			if (!mShowDistanceLine)
			{
				auto keys = mpLogics->pVisLogic->getAbstractObjectKeys();
				if (std::find(keys.begin(), keys.end(), "distanceLine") != keys.end())
				{
					mpLogics->pVisLogic->deleteAbstractObject("distanceLine");
				}
			}
			mpLogics->pVisLogic->render();
		}

		/** \brief Toggles between showing various helpers to display the axes of the coordinate system.
		
		0: No helpers
		1: 3d axes 
		2: 3d planes 
		*/
		void InteractorStyle3D::toggleAxes()
		{
			mAxesDisplayMode = (mAxesDisplayMode + 1) % 3;

			// delete the objects from the old mode
			if (mAxesDisplayMode == 0)
			{
				LOG_LINE << "Axes drawing is: nothing";
				auto keys = mpLogics->pVisLogic->getAbstractObjectKeys();
				if (std::find(keys.begin(), keys.end(), "zPlane") != keys.end())
				{
					mpLogics->pVisLogic->deleteAbstractObject("zPlane");
				}
				if (std::find(keys.begin(), keys.end(), "xPlane") != keys.end())
				{
					mpLogics->pVisLogic->deleteAbstractObject("xPlane");
				}
				if (std::find(keys.begin(), keys.end(), "yPlane") != keys.end())
				{
					mpLogics->pVisLogic->deleteAbstractObject("yPlane");
				}
			}
			else if (mAxesDisplayMode == 2)
			{
				LOG_LINE << "Axes drawing is: planes";
				auto keys = mpLogics->pVisLogic->getAbstractObjectKeys();
				if (std::find(keys.begin(), keys.end(), "3dAxes") != keys.end())
				{
					mpLogics->pVisLogic->deleteAbstractObject("3dAxes");
				}
			}
			else
			{
				LOG_LINE << "Axes drawing is: 3d axes";
			}
			mpLogics->pVisLogic->render();
		}

		

		/** \brief Deletes existing VisualObjects used for displaying the distanceLine, 3dAxes or planes
		*/
		void InteractorStyle3D::removeNavigationHelpers()
		{
			auto keys = mpLogics->pVisLogic->getAbstractObjectKeys();
			std::vector<std::string> keysToDelete = { "distanceLine", "3dAxes", "zPlane", "xPlane", "yPlane" };

			for (auto& s : keysToDelete)
			{
				if (std::find(keys.begin(), keys.end(), s) != keys.end())
				{
					mpLogics->pVisLogic->deleteAbstractObject(s);
				}
			}
		}
	}
}

