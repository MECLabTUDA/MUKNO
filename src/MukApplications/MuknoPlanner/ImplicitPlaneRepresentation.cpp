#include "private/muk.pch"
#include "private/ImplicitPlaneRepresentation.h"


#include <vtkAssemblyPaths.h>
#include <vtkCamera.h>
#include <vtkInteractorObserver.h>
#include <vtkBox.h>
#include <vtkTransform.h>
#include <vtkObject.h>

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(ImplicitPlaneRepresentation);

  /**
	  Default Initialisation of the Representation
  */
  ImplicitPlaneRepresentation::ImplicitPlaneRepresentation()
  {
	  // Create Arrows as a Startpoint for the Pathplaning
	  // Arrow 1
	  this->mpArrowMotionPlanningConeSource1 = vtkConeSource::New();
	  this->mpArrowMotionPlanningConeSource1->SetResolution(12);
	  this->mpArrowMotionPlanningConeSource1->SetAngle(25.0);
	  this->mpArrowMotionPlanningConeMapper1 = vtkPolyDataMapper::New();
	  this->mpArrowMotionPlanningConeMapper1->SetInputConnection(this->mpArrowMotionPlanningConeSource1->GetOutputPort());
	  this->mpArrowMotionPlanningConeActor1 = vtkActor::New();
	  this->mpArrowMotionPlanningConeActor1->SetMapper(this->mpArrowMotionPlanningConeMapper1);

	  // Arrow 2
	  this->mpArrowMotionPlanningConeSource2 = vtkConeSource::New();
	  this->mpArrowMotionPlanningConeSource2->SetResolution(12);
	  this->mpArrowMotionPlanningConeSource2->SetAngle(25.0);
	  this->mpArrowMotionPlanningConeMapper2 = vtkPolyDataMapper::New();
	  this->mpArrowMotionPlanningConeMapper2->SetInputConnection(this->mpArrowMotionPlanningConeSource2->GetOutputPort());
	  this->mpArrowMotionPlanningConeActor2 = vtkActor::New();
	  this->mpArrowMotionPlanningConeActor2->SetMapper(this->mpArrowMotionPlanningConeMapper2);
	
	  // Arrow 3
	  this->mpArrowMotionPlanningConeSource3 = vtkConeSource::New();
	  this->mpArrowMotionPlanningConeSource3->SetResolution(12);
	  this->mpArrowMotionPlanningConeSource3->SetAngle(25.0);
	  this->mpArrowMotionPlanningConeMapper3 = vtkPolyDataMapper::New();
	  this->mpArrowMotionPlanningConeMapper3->SetInputConnection(this->mpArrowMotionPlanningConeSource3->GetOutputPort());
	  this->mpArrowMotionPlanningConeActor3 = vtkActor::New();
	  this->mpArrowMotionPlanningConeActor3->SetMapper(this->mpArrowMotionPlanningConeMapper3);

	  // Line Arrow1
	  this->mpArrowMotionPlanningLineSource1 = vtkLineSource::New();
	  this->mpArrowMotionPlanningLineSource1->SetResolution(1);
	  this->mpArrowMotionPlanningLineMapper1 = vtkPolyDataMapper::New();
	  this->mpArrowMotionPlanningLineMapper1->SetInputConnection(this->mpArrowMotionPlanningLineSource1->GetOutputPort());
	  this->mpArrowMotionPlanningLineActor1 = vtkActor::New();
	  this->mpArrowMotionPlanningLineActor1->SetMapper(this->mpArrowMotionPlanningLineMapper1);

	  // Line Arrow2
	  this->mpArrowMotionPlanningLineSource2 = vtkLineSource::New();
	  this->mpArrowMotionPlanningLineSource2->SetResolution(1);
	  this->mpArrowMotionPlanningLineMapper2 = vtkPolyDataMapper::New();
	  this->mpArrowMotionPlanningLineMapper2->SetInputConnection(this->mpArrowMotionPlanningLineSource2->GetOutputPort());
	  this->mpArrowMotionPlanningLineActor2 = vtkActor::New();
	  this->mpArrowMotionPlanningLineActor2->SetMapper(this->mpArrowMotionPlanningLineMapper2);

	  // Line Arrow3
	  this->mpArrowMotionPlanningLineSource3 = vtkLineSource::New();
	  this->mpArrowMotionPlanningLineSource3->SetResolution(1);
	  this->mpArrowMotionPlanningLineMapper3 = vtkPolyDataMapper::New();
	  this->mpArrowMotionPlanningLineMapper3->SetInputConnection(this->mpArrowMotionPlanningLineSource3->GetOutputPort());
	  this->mpArrowMotionPlanningLineActor3 = vtkActor::New();
	  this->mpArrowMotionPlanningLineActor3->SetMapper(this->mpArrowMotionPlanningLineMapper3);

	  // Set default direction of the Arrows, so there is an initialized value
	  this->mpArrowMotionPlanningConeSource1->SetDirection(1, 2, 2);
	  this->mpArrowMotionPlanningConeSource2->SetDirection(2, 2, 1);
	  this->mpArrowMotionPlanningConeSource3->SetDirection(2, 1, 2);

	  // Initialize the RotationMatrix
	  vtkMath::Identity3x3(mRotateMatrix);

	  // Manage the picking stuff of the Arrows
	  this->Picker->AddPickList(this->mpArrowMotionPlanningLineActor1);
	  this->Picker->AddPickList(this->mpArrowMotionPlanningConeActor1);
	  this->Picker->AddPickList(this->mpArrowMotionPlanningLineActor2);
	  this->Picker->AddPickList(this->mpArrowMotionPlanningConeActor2);
	  this->Picker->AddPickList(this->mpArrowMotionPlanningLineActor3);
	  this->Picker->AddPickList(this->mpArrowMotionPlanningConeActor3);

	  this->CreateDefaultProperties();

	  // Pass the initial properties to the actors
	  this->mpArrowMotionPlanningLineActor1->SetProperty(this->mpConeProperty);
	  this->mpArrowMotionPlanningConeActor1->SetProperty(this->mpConeProperty);
	  this->mpArrowMotionPlanningLineActor2->SetProperty(this->mpConeProperty);
	  this->mpArrowMotionPlanningConeActor2->SetProperty(this->mpConeProperty);
	  this->mpArrowMotionPlanningLineActor3->SetProperty(this->mpConeProperty);
	  this->mpArrowMotionPlanningConeActor3->SetProperty(this->mpConeProperty);
  }

  /**
    Default destruction of the Representation
  */
  ImplicitPlaneRepresentation::~ImplicitPlaneRepresentation() 
  {
	  this->mpArrowMotionPlanningLineSource1->Delete();
	  this->mpArrowMotionPlanningLineMapper1->Delete();
	  this->mpArrowMotionPlanningLineActor1->Delete();

	  this->mpArrowMotionPlanningConeSource1->Delete();
	  this->mpArrowMotionPlanningConeMapper1->Delete();
	  this->mpArrowMotionPlanningConeActor1->Delete();

	  this->mpArrowMotionPlanningLineSource2->Delete();
	  this->mpArrowMotionPlanningLineMapper2->Delete();
	  this->mpArrowMotionPlanningLineActor2->Delete();

	  this->mpArrowMotionPlanningConeSource2->Delete();
	  this->mpArrowMotionPlanningConeMapper2->Delete();
	  this->mpArrowMotionPlanningConeActor2->Delete();

	  this->mpArrowMotionPlanningLineSource3->Delete();
	  this->mpArrowMotionPlanningLineMapper3->Delete();
	  this->mpArrowMotionPlanningLineActor3->Delete();

	  this->mpArrowMotionPlanningConeSource3->Delete();
	  this->mpArrowMotionPlanningConeMapper3->Delete();
	  this->mpArrowMotionPlanningConeActor3->Delete();

	  this->mpConeProperty->Delete();
	  this->mpSelectedConeProperty->Delete();
  };

  /**
   *
   *\brif checks wich Interactionstate is activated
   *
   * @param X x-coordinate of the mouse
   * @param Y y-coordinate of the mouse
   * 
   */
  int ImplicitPlaneRepresentation::ComputeInteractionState(int X, int Y, int vtkNotUsed(modify))
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
		  else if (prop == this->mpArrowMotionPlanningConeActor1 || prop == this->mpArrowMotionPlanningLineActor1)
		  {
			  this->InteractionState = ImplicitPlaneRepresentation::RotatingArrow1;
			  this->SetRepresentationState(ImplicitPlaneRepresentation::RotatingArrow1);
		  }
		  else if (prop == this->mpArrowMotionPlanningConeActor2 || prop == this->mpArrowMotionPlanningLineActor2)
		  {
			  this->InteractionState = ImplicitPlaneRepresentation::RotatingArrow2;
			  this->SetRepresentationState(ImplicitPlaneRepresentation::RotatingArrow2);
		  }
		  else if (prop == this->mpArrowMotionPlanningConeActor3 || prop == this->mpArrowMotionPlanningLineActor3)
		  {
			  this->InteractionState = ImplicitPlaneRepresentation::RotatingArrow3;
			  this->SetRepresentationState(ImplicitPlaneRepresentation::RotatingArrow3);
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
  void ImplicitPlaneRepresentation::BuildRepresentation()
  {
	  if (!this->Renderer || !this->Renderer->GetRenderWindow())
	  {
		  return;
	  }

	  // rotated direction used to set the direction of the arrows
	  double rotated[3];

	  vtkInformation *info = this->GetPropertyKeys();
	  this->OutlineActor->SetPropertyKeys(info);
	  this->CutActor->SetPropertyKeys(info);
	  this->EdgesActor->SetPropertyKeys(info);
	  this->ConeActor->SetPropertyKeys(info);
	  this->LineActor->SetPropertyKeys(info);
	  this->ConeActor2->SetPropertyKeys(info);
	  this->LineActor2->SetPropertyKeys(info);
	  this->SphereActor->SetPropertyKeys(info);
	  this->mpArrowMotionPlanningConeActor1->SetPropertyKeys(info);
	  this->mpArrowMotionPlanningLineActor1->SetPropertyKeys(info);
	  this->mpArrowMotionPlanningConeActor2->SetPropertyKeys(info);
	  this->mpArrowMotionPlanningLineActor2->SetPropertyKeys(info);
	  this->mpArrowMotionPlanningConeActor3->SetPropertyKeys(info);
	  this->mpArrowMotionPlanningLineActor3->SetPropertyKeys(info);

	  if (this->GetMTime() > this->BuildTime ||
		  this->Plane->GetMTime() > this->BuildTime ||
		  this->Renderer->GetRenderWindow()->GetMTime() > this->BuildTime||
		  mRadius != mOldRadius)
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
		  double d = mRadius*1.2;

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

		  // Calculates the rotation for Arrow1
		  // rotated = new direction of the rotated Arrow
		  vtkMath::Multiply3x3(mRotateMatrix, this->mpArrowMotionPlanningConeSource1->GetDirection(), rotated);
		  vtkMath::Normalize(rotated);

		  p2[0] = origin[0] + d * rotated[0];
		  p2[1] = origin[1] + d * rotated[1];
		  p2[2] = origin[2] + d * rotated[2];

		  this->mpArrowMotionPlanningLineSource1->SetPoint1(origin);
		  this->mpArrowMotionPlanningLineSource1->SetPoint2(p2);
		  this->mpArrowMotionPlanningConeSource1->SetCenter(p2);
		  this->mpArrowMotionPlanningConeSource1->SetDirection(rotated);
		
		  // Calculates the rotation for Arrow2
		  // rotated = new direction of the rotated Arrow
		  vtkMath::Multiply3x3(mRotateMatrix, this->mpArrowMotionPlanningConeSource2->GetDirection(), rotated);
		  vtkMath::Normalize(rotated);

		  p2[0] = origin[0] + d * rotated[0];
		  p2[1] = origin[1] + d * rotated[1];
		  p2[2] = origin[2] + d * rotated[2];

		  this->mpArrowMotionPlanningLineSource2->SetPoint1(origin);
		  this->mpArrowMotionPlanningLineSource2->SetPoint2(p2);
		  this->mpArrowMotionPlanningConeSource2->SetCenter(p2);
		  this->mpArrowMotionPlanningConeSource2->SetDirection(rotated);

		  // Calculates the rotation for Arrow3
		  // rotated = new direction of the rotated Arrow
		  vtkMath::Multiply3x3(mRotateMatrix, this->mpArrowMotionPlanningConeSource3->GetDirection(), rotated);
		  vtkMath::Normalize(rotated);

		  p2[0] = origin[0] + d * rotated[0];
		  p2[1] = origin[1] + d * rotated[1];
		  p2[2] = origin[2] + d * rotated[2];

		  this->mpArrowMotionPlanningLineSource3->SetPoint1(origin);
		  this->mpArrowMotionPlanningLineSource3->SetPoint2(p2);
		  this->mpArrowMotionPlanningConeSource3->SetCenter(p2);
		  this->mpArrowMotionPlanningConeSource3->SetDirection(rotated);

		  // resets the rotationmatrix to avoid further rotation of Arrows
		  vtkMath::Identity3x3(mRotateMatrix);

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
		  this->mOldRadius = mRadius;
		  this->SizeHandles();
		  this->BuildTime.Modified();
	  }

  }

  /**
   * \brief Manages the the size of the Cones if Zoomed
   */
  void ImplicitPlaneRepresentation::SizeHandles()
  {
	  // radius scales with the zoom to pick actors better (not used for cones but it may be changed here)
	  double radius =
		  this->vtkWidgetRepresentation::SizeHandlesInPixels(1.5, this->Sphere->GetCenter());
	  // defaultRadius for the Arrows' Cones (change to scale)
	  double defaultRadius = mRadius * 0.2;
	  this->ConeSource->SetHeight(defaultRadius);
	  this->ConeSource->SetRadius(defaultRadius);
	  this->ConeSource2->SetHeight(defaultRadius);
	  this->ConeSource2->SetRadius(defaultRadius);
	  this->mpArrowMotionPlanningConeSource1->SetHeight(defaultRadius);
	  this->mpArrowMotionPlanningConeSource1->SetRadius(defaultRadius);
	  this->mpArrowMotionPlanningConeSource2->SetHeight(defaultRadius);
	  this->mpArrowMotionPlanningConeSource2->SetRadius(defaultRadius);
	  this->mpArrowMotionPlanningConeSource3->SetHeight(defaultRadius);
	  this->mpArrowMotionPlanningConeSource3->SetRadius(defaultRadius);
	
	  this->Sphere->SetRadius(mRadius);
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
  void ImplicitPlaneRepresentation::Rotate(double X, double Y,
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
  }


  /** \brief Rotates a certain cone
  
    \param X mouse position on screen
    \param Y mouse position on screen
    \param p1 last mouse position (world)
    \param p2 new mouse position (world)
    \param coneDirection
  */
  void ImplicitPlaneRepresentation::RotateCone(double X, double Y, double * p1, double * p2, double * coneDirection)
  {
	  double v[3]; //vector of motion
	  double axis[3]; //axis of rotation
	  double theta; //rotation angle

	  // mouse motion vector in world space
	  v[0] = p2[0] - p1[0];
	  v[1] = p2[1] - p1[1];
	  v[2] = p2[2] - p1[2];

	  double *origin = this->Plane->GetOrigin();

	  double *conesDirection;
	  if (this->InteractionState == ImplicitPlaneRepresentation::RotatingArrow1)
		  conesDirection = this->mpArrowMotionPlanningConeSource1->GetDirection();
	  else if (this->InteractionState == ImplicitPlaneRepresentation::RotatingArrow2)
		  conesDirection = this->mpArrowMotionPlanningConeSource2->GetDirection();
	  else if (this->InteractionState == ImplicitPlaneRepresentation::RotatingArrow3)
		  conesDirection = this->mpArrowMotionPlanningConeSource3->GetDirection();

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
	  this->Transform->TransformNormal(conesDirection, nNew);
	
	  if (this->InteractionState == ImplicitPlaneRepresentation::RotatingArrow1)
		  this->mpArrowMotionPlanningConeSource1->SetDirection(nNew);
	  else if (this->InteractionState == ImplicitPlaneRepresentation::RotatingArrow2)
		  this->mpArrowMotionPlanningConeSource2->SetDirection(nNew);
	  else if (this->InteractionState == ImplicitPlaneRepresentation::RotatingArrow3)
		  this->mpArrowMotionPlanningConeSource3->SetDirection(nNew);
	  this->Modified();
  }

  /** \brief Scales Origin and Box
  */
  void ImplicitPlaneRepresentation::Scale(double *p1, double *p2,
	  double vtkNotUsed(X), double Y, double * vpn)
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

  /**
   * \brief Set the normal to the plane and calculates the rotation of the normal
   *
   * @param x, y, z New normalvector
   */
  void ImplicitPlaneRepresentation::SetNormal(double x, double y, double z)
  {
	  double n[3], n2[3];
	  n[0] = x;
	  n[1] = y;
	  n[2] = z;
	  vtkMath::Normalize(n);

	  this->Plane->GetNormal(n2);
	  if (n[0] != n2[0] || n[1] != n2[1] || n[2] != n2[2])
	  {
		  // Compute rotation matrix between two vectors
		  double vec[3];
		  vtkMath::Cross(n, n2, vec);
		  double costheta = vtkMath::Dot(n, n2);
		  double sintheta = vtkMath::Norm(vec);
		  double theta = atan2(sintheta, costheta);
		  if (sintheta != 0)
		  {
			  vec[0] /= sintheta;
			  vec[1] /= sintheta;
			  vec[2] /= sintheta;
		  }
		  // convert to quaternion
		  costheta = cos(0.5*theta);
		  sintheta = sin(0.5*theta);
		  double quat[4];
		  quat[0] = costheta;
		  quat[1] = vec[0] * sintheta;
		  quat[2] = vec[1] * sintheta;
		  quat[3] = vec[2] * sintheta;
		  // convert to matrix
		  vtkMath::QuaternionToMatrix3x3(quat, mRotateMatrix);
		  vtkMath::Invert3x3(mRotateMatrix, mRotateMatrix);
		  this->Plane->SetNormal(n);
		  this->Modified();
	  }
  }

  /**
   * \brief LockNormalToCamera will cause the normal to follow the camera's normal
   *
   * @param lock Wether Camera is locked or not
   */
  void ImplicitPlaneRepresentation::SetLockNormalToCamera(int lock)
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
		  this->Picker->DeletePickList(this->mpArrowMotionPlanningLineActor1);
		  this->Picker->DeletePickList(this->mpArrowMotionPlanningConeActor1);
		  this->Picker->DeletePickList(this->mpArrowMotionPlanningLineActor2);
		  this->Picker->DeletePickList(this->mpArrowMotionPlanningConeActor2);
		  this->Picker->DeletePickList(this->mpArrowMotionPlanningLineActor3);
		  this->Picker->DeletePickList(this->mpArrowMotionPlanningConeActor3);
		  this->Picker->DeletePickList(this->SphereActor);

		  this->SetNormalToCamera();
	  }
	  else
	  {
		  this->Picker->AddPickList(this->LineActor);
		  this->Picker->AddPickList(this->ConeActor);
		  this->Picker->AddPickList(this->LineActor2);
		  this->Picker->AddPickList(this->ConeActor2);
		  this->Picker->AddPickList(this->mpArrowMotionPlanningLineActor1);
		  this->Picker->AddPickList(this->mpArrowMotionPlanningConeActor1);
		  this->Picker->AddPickList(this->mpArrowMotionPlanningLineActor2);
		  this->Picker->AddPickList(this->mpArrowMotionPlanningConeActor2);
		  this->Picker->AddPickList(this->mpArrowMotionPlanningLineActor3);
		  this->Picker->AddPickList(this->mpArrowMotionPlanningConeActor3);
		  this->Picker->AddPickList(this->SphereActor);
	  }

	  this->LockNormalToCamera = lock;
	  this->Modified();
  }

  /**
   * \brief Highights the Objects wether it's their Representationstate
   * 
   * @param state The new Representationstate
   */
  void ImplicitPlaneRepresentation::SetRepresentationState(int state)
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
	  else if (state == ImplicitPlaneRepresentation::RotatingArrow1)
	  {
		  this->HighlightNormal(1);
		  this->HighlightPlane(1);
	  }
	  // Highlights for rotating Arrows
	  else if (state == ImplicitPlaneRepresentation::RotatingArrow2)
	  {
		  this->HighlightNormal(1);
		  this->HighlightPlane(1);
	  }
	  // Highlights for rotating Arrows
	  else if (state == ImplicitPlaneRepresentation::RotatingArrow3)
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
  void ImplicitPlaneRepresentation::WidgetInteraction(double e[2])
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
	  else if (this->InteractionState == ImplicitPlaneRepresentation::Scaling &&
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
	  else if (this->InteractionState == ImplicitPlaneRepresentation::RotatingArrow1)
	  {

		  vpn[0] = this->mpArrowMotionPlanningConeSource1->GetDirection()[0];
		  vpn[1] = this->mpArrowMotionPlanningConeSource1->GetDirection()[1];
		  vpn[2] = this->mpArrowMotionPlanningConeSource1->GetDirection()[2];
		  this->RotateCone(e[0], e[1], prevPickPoint, pickPoint, vpn);
	  }
	  else if (this->InteractionState == ImplicitPlaneRepresentation::RotatingArrow2)
	  {
		  vpn[0] = this->mpArrowMotionPlanningConeSource2->GetDirection()[0];
		  vpn[1] = this->mpArrowMotionPlanningConeSource2->GetDirection()[1];
		  vpn[2] = this->mpArrowMotionPlanningConeSource2->GetDirection()[2];
		  this->RotateCone(e[0], e[1], prevPickPoint, pickPoint, vpn);
	  }
	  else if (this->InteractionState == ImplicitPlaneRepresentation::RotatingArrow3)
	  {
		  vpn[0] = this->mpArrowMotionPlanningConeSource3->GetDirection()[0];
		  vpn[1] = this->mpArrowMotionPlanningConeSource3->GetDirection()[1];
		  vpn[2] = this->mpArrowMotionPlanningConeSource3->GetDirection()[2];
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
  double *ImplicitPlaneRepresentation::GetBounds()
  {
	  this->BuildRepresentation();
	  this->BoundingBox->SetBounds(this->OutlineActor->GetBounds());
	  this->BoundingBox->AddBounds(this->CutActor->GetBounds());
	  this->BoundingBox->AddBounds(this->EdgesActor->GetBounds());
	  this->BoundingBox->AddBounds(this->ConeActor->GetBounds());
	  this->BoundingBox->AddBounds(this->LineActor->GetBounds());
	  this->BoundingBox->AddBounds(this->ConeActor2->GetBounds());
	  this->BoundingBox->AddBounds(this->LineActor2->GetBounds());
	  this->BoundingBox->AddBounds(this->mpArrowMotionPlanningConeActor1->GetBounds());
	  this->BoundingBox->AddBounds(this->mpArrowMotionPlanningLineActor1->GetBounds());
	  this->BoundingBox->AddBounds(this->mpArrowMotionPlanningConeActor2->GetBounds());
	  this->BoundingBox->AddBounds(this->mpArrowMotionPlanningLineActor2->GetBounds());
	  this->BoundingBox->AddBounds(this->mpArrowMotionPlanningConeActor3->GetBounds());
	  this->BoundingBox->AddBounds(this->mpArrowMotionPlanningLineActor3->GetBounds());
	  this->BoundingBox->AddBounds(this->SphereActor->GetBounds());

	  return this->BoundingBox->GetBounds();
  }

  /**
   * \brief returns the actors
   * 
   * @param pc Output to get the actors
   */
  void ImplicitPlaneRepresentation::GetActors(vtkPropCollection *pc)
  {
	  this->OutlineActor->GetActors(pc);
	  this->CutActor->GetActors(pc);
	  this->EdgesActor->GetActors(pc);
	  this->ConeActor->GetActors(pc);
	  this->LineActor->GetActors(pc);
	  this->ConeActor2->GetActors(pc);
	  this->LineActor2->GetActors(pc);
	  this->mpArrowMotionPlanningConeActor1->GetActors(pc);
	  this->mpArrowMotionPlanningLineActor1->GetActors(pc);
	  this->mpArrowMotionPlanningConeActor2->GetActors(pc);
	  this->mpArrowMotionPlanningLineActor2->GetActors(pc);
	  this->mpArrowMotionPlanningConeActor3->GetActors(pc);
	  this->mpArrowMotionPlanningLineActor3->GetActors(pc);
	  this->SphereActor->GetActors(pc);
  }

  /** \brief Release graphics resources and ask components to release their own resources
  */
  void ImplicitPlaneRepresentation::ReleaseGraphicsResources(vtkWindow *w)
  {
	  this->OutlineActor->ReleaseGraphicsResources(w);
	  this->CutActor->ReleaseGraphicsResources(w);
	  this->EdgesActor->ReleaseGraphicsResources(w);
	  this->ConeActor->ReleaseGraphicsResources(w);
	  this->LineActor->ReleaseGraphicsResources(w);
	  this->ConeActor2->ReleaseGraphicsResources(w);
	  this->LineActor2->ReleaseGraphicsResources(w);
	  this->mpArrowMotionPlanningConeActor1->ReleaseGraphicsResources(w);
	  this->mpArrowMotionPlanningLineActor1->ReleaseGraphicsResources(w);
	  this->mpArrowMotionPlanningConeActor2->ReleaseGraphicsResources(w);
	  this->mpArrowMotionPlanningLineActor2->ReleaseGraphicsResources(w);
	  this->mpArrowMotionPlanningConeActor3->ReleaseGraphicsResources(w);
	  this->mpArrowMotionPlanningLineActor3->ReleaseGraphicsResources(w);
	  this->SphereActor->ReleaseGraphicsResources(w);
  }

  // Support the standard render methods. 
  int ImplicitPlaneRepresentation::RenderOpaqueGeometry(vtkViewport *v)
  {
	  int count = 0;
	  this->BuildRepresentation();
	  count += this->OutlineActor->RenderOpaqueGeometry(v);
	  count += this->EdgesActor->RenderOpaqueGeometry(v);
	  if (!this->LockNormalToCamera)
	  {
		  count += this->ConeActor->RenderOpaqueGeometry(v);
		  count += this->LineActor->RenderOpaqueGeometry(v);
		  count += this->ConeActor2->RenderOpaqueGeometry(v);
		  count += this->LineActor2->RenderOpaqueGeometry(v);
		  count += this->mpArrowMotionPlanningConeActor1->RenderOpaqueGeometry(v);
		  count += this->mpArrowMotionPlanningLineActor1->RenderOpaqueGeometry(v);
		  count += this->mpArrowMotionPlanningConeActor2->RenderOpaqueGeometry(v);
		  count += this->mpArrowMotionPlanningLineActor2->RenderOpaqueGeometry(v);
		  count += this->mpArrowMotionPlanningConeActor3->RenderOpaqueGeometry(v);
		  count += this->mpArrowMotionPlanningLineActor3->RenderOpaqueGeometry(v);
		  count += this->SphereActor->RenderOpaqueGeometry(v);
	  }
	  if (this->DrawPlane)
	  {
		  count += this->CutActor->RenderOpaqueGeometry(v);
	  }

	  return count;
  }

  // Support the standard render methods. 
  int ImplicitPlaneRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport *v)
  {
	  int count = 0;
	  this->BuildRepresentation();
	  count += this->OutlineActor->RenderTranslucentPolygonalGeometry(v);
	  count += this->EdgesActor->RenderTranslucentPolygonalGeometry(v);
	  if (!this->LockNormalToCamera)
	  {
		  count += this->ConeActor->RenderTranslucentPolygonalGeometry(v);
		  count += this->LineActor->RenderTranslucentPolygonalGeometry(v);
		  count += this->ConeActor2->RenderTranslucentPolygonalGeometry(v);
		  count += this->LineActor2->RenderTranslucentPolygonalGeometry(v);
		  count += this->mpArrowMotionPlanningConeActor1->RenderTranslucentPolygonalGeometry(v);
		  count += this->mpArrowMotionPlanningLineActor1->RenderTranslucentPolygonalGeometry(v);
		  count += this->mpArrowMotionPlanningConeActor2->RenderTranslucentPolygonalGeometry(v);
		  count += this->mpArrowMotionPlanningLineActor2->RenderTranslucentPolygonalGeometry(v);
		  count += this->mpArrowMotionPlanningConeActor3->RenderTranslucentPolygonalGeometry(v);
		  count += this->mpArrowMotionPlanningLineActor3->RenderTranslucentPolygonalGeometry(v);
		  count += this->SphereActor->RenderTranslucentPolygonalGeometry(v);
	  }
	  if (this->DrawPlane)
	  {
		  count += this->CutActor->RenderTranslucentPolygonalGeometry(v);
	  }

	  return count;
  }

  // Support the standard render methods. 
  int ImplicitPlaneRepresentation::HasTranslucentPolygonalGeometry()
  {
	  int result = 0;
	  result |= this->OutlineActor->HasTranslucentPolygonalGeometry();
	  result |= this->EdgesActor->HasTranslucentPolygonalGeometry();
	  if (!this->LockNormalToCamera)
	  {
		  result |= this->ConeActor->HasTranslucentPolygonalGeometry();
		  result |= this->LineActor->HasTranslucentPolygonalGeometry();
		  result |= this->ConeActor2->HasTranslucentPolygonalGeometry();
		  result |= this->LineActor2->HasTranslucentPolygonalGeometry();
		  result |= this->mpArrowMotionPlanningConeActor1->HasTranslucentPolygonalGeometry();
		  result |= this->mpArrowMotionPlanningLineActor1->HasTranslucentPolygonalGeometry();
		  result |= this->mpArrowMotionPlanningConeActor2->HasTranslucentPolygonalGeometry();
		  result |= this->mpArrowMotionPlanningLineActor2->HasTranslucentPolygonalGeometry();
		  result |= this->mpArrowMotionPlanningConeActor3->HasTranslucentPolygonalGeometry();
		  result |= this->mpArrowMotionPlanningLineActor3->HasTranslucentPolygonalGeometry();
		  result |= this->SphereActor->HasTranslucentPolygonalGeometry();
	  }
	  if (this->DrawPlane)
	  {
		  result |= this->CutActor->HasTranslucentPolygonalGeometry();
	  }

	  return result;
  }

  //----------------------------------------------------------------------------
  // used to print the state
  //----------------------------------------------------------------------------
  void ImplicitPlaneRepresentation::PrintSelf(ostream& os, vtkIndent indent)
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

	  if (this->mpConeProperty)
	  {
		  os << indent << "Cone Property: " << this->mpConeProperty << "\n";
	  }
	  else
	  {
		  os << indent << "Cone Property: (none)\n";
	  }
	  if (this->mpSelectedConeProperty)
	  {
		  os << indent << "Selected Cone Property: "
			  << this->mpSelectedConeProperty << "\n";
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
  void ImplicitPlaneRepresentation::CreateDefaultProperties()
  {
	  // Normal properties
	  this->NormalProperty = vtkProperty::New();
	  this->NormalProperty->SetColor(1, 1, 1);
	  this->NormalProperty->SetLineWidth(2);

	  this->SelectedNormalProperty = vtkProperty::New();
	  this->SelectedNormalProperty->SetColor(1, 0, 0);
	  this->NormalProperty->SetLineWidth(2);

	  // ArrowProperties
	  this->mpConeProperty = vtkProperty::New();
	  this->mpConeProperty->SetColor(0, 255, 0);
	  this->mpConeProperty->SetLineWidth(2);

	  this->mpSelectedConeProperty = vtkProperty::New();
	  this->mpSelectedConeProperty->SetColor(0, 1, 0);
	  this->mpSelectedConeProperty->SetLineWidth(2);

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
  void ImplicitPlaneRepresentation::HighlightNormal(int highlight)
  {
	  if (highlight)
	  {
		  this->LineActor->SetProperty(this->SelectedNormalProperty);
		  this->ConeActor->SetProperty(this->SelectedNormalProperty);
		  this->LineActor2->SetProperty(this->SelectedNormalProperty);
		  this->ConeActor2->SetProperty(this->SelectedNormalProperty);
		  this->mpArrowMotionPlanningLineActor1->SetProperty(this->mpSelectedConeProperty);
		  this->mpArrowMotionPlanningConeActor1->SetProperty(this->mpSelectedConeProperty);
		  this->mpArrowMotionPlanningLineActor2->SetProperty(this->mpSelectedConeProperty);
		  this->mpArrowMotionPlanningConeActor2->SetProperty(this->mpSelectedConeProperty);
		  this->mpArrowMotionPlanningLineActor3->SetProperty(this->mpSelectedConeProperty);
		  this->mpArrowMotionPlanningConeActor3->SetProperty(this->mpSelectedConeProperty);
		  this->SphereActor->SetProperty(this->SelectedNormalProperty);
	  }
	  else
	  {
		  this->LineActor->SetProperty(this->NormalProperty);
		  this->ConeActor->SetProperty(this->NormalProperty);
		  this->LineActor2->SetProperty(this->NormalProperty);
		  this->ConeActor2->SetProperty(this->NormalProperty);
		  this->mpArrowMotionPlanningLineActor1->SetProperty(this->mpConeProperty);
		  this->mpArrowMotionPlanningConeActor1->SetProperty(this->mpConeProperty);
		  this->mpArrowMotionPlanningLineActor2->SetProperty(this->mpConeProperty);
		  this->mpArrowMotionPlanningConeActor2->SetProperty(this->mpConeProperty);
		  this->mpArrowMotionPlanningLineActor3->SetProperty(this->mpConeProperty);
		  this->mpArrowMotionPlanningConeActor3->SetProperty(this->mpConeProperty);
		  this->SphereActor->SetProperty(this->NormalProperty);
	  }
  }

}
}