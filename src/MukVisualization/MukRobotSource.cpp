#include "private/muk.pch"
#include "MukRobotSource.h"

#include <vtkInformationVector.h>
#include <vtkInformation.h>
#include <vtkSmartPointer.h>
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkAppendPolyData.h"
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkPlaneCollection.h>
#include <vtkTransform.h>
#include <vtkClipClosedSurface.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkLineSource.h>
#include <vtkTubeFilter.h>
#include <vtkDoubleArray.h>
#include <vtkCellData.h>

#define _USE_MATH_DEFINES
#include <math.h>

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(MukRobotSource);

  /**
  */
  MukRobotSource::MukRobotSource(int res) 
  {
    this->SetNumberOfInputPorts(0);
    this->Resolution = res;
    this->Center[0] = this->Center[1] = this->Center[2] = 0.0;
    this->Angle = 0;

    this->LargeCylinderHeight = 1.85;
    this->LargeCylinderRadius = 1.225;
    this->LargeCylinderRGB[0] = 255;
    this->LargeCylinderRGB[1] = 143;
    this->LargeCylinderRGB[2] = 31;

    this->PadRadius = 0.75;
    this->PadRGB[0] = 252;
    this->PadRGB[1] = 0;
    this->PadRGB[2] = 146;

    this->HeadLength = 1.7;
    this->HeadRGB[0] = 164;
    this->HeadRGB[1] = 164;
    this->HeadRGB[2] = 164;

    this->EngineCylinderHeight = 0.665;
    this->EngineCylinderRadius = 0.55;
    this->EngineRGB[0] = 58;
    this->EngineRGB[1] = 211;
    this->EngineRGB[2] = 41;

    this->BellowCylinderHeight = 2.7;
    this->BellowCylinderRadius = 0.55;
    this->BellowCylinderRGB[0] = 253;
    this->BellowCylinderRGB[1] = 247;
    this->BellowCylinderRGB[2] = 6;

    this->BellowPointsN = 10;
  }

  /**
  */
  int MukRobotSource::RequestData(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector)
  {
    // get the info object
    vtkInformation *outInfo = outputVector->GetInformationObject(0);
    // get the ouptut
    vtkPolyData *output = vtkPolyData::SafeDownCast(
      outInfo->Get(vtkDataObject::DATA_OBJECT()));

    // transformfilter for any transformation
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

    // the first large cylinder, all positions based on this cylinder
    vtkCylinderSource * big_cyl_1 = vtkCylinderSource::New();
    big_cyl_1->SetRadius(this->LargeCylinderRadius);
    big_cyl_1->SetHeight(this->LargeCylinderHeight);
    big_cyl_1->SetResolution(this->Resolution);
    big_cyl_1->SetCenter(this->Center[0], this->Center[1], this->Center[2]);
    big_cyl_1->Update();

    // 3 pad spheres
    // the first sphere can be initialized anywhere on the surface of the first cylinder
    vtkSmartPointer<vtkSphereSource> pad_sphere_1 = vtkSmartPointer<vtkSphereSource>::New();
    pad_sphere_1->SetCenter(big_cyl_1->GetRadius(), 0, 0);
    pad_sphere_1->SetRadius(this->PadRadius);
    pad_sphere_1->SetThetaResolution(this->Resolution);
    pad_sphere_1->SetPhiResolution(this->Resolution);

    // the second one needs to be rotated around the cylinder (120) 
    // apply the rotation then translate
    vtkSmartPointer<vtkSphereSource> pad_sphere_2 = vtkSmartPointer<vtkSphereSource>::New();
    pad_sphere_2->SetCenter(pad_sphere_1->GetCenter()[0], pad_sphere_1->GetCenter()[1], pad_sphere_1->GetCenter()[2]);
    pad_sphere_2->SetRadius(this->PadRadius);
    pad_sphere_2->SetThetaResolution(this->Resolution);
    pad_sphere_2->SetPhiResolution(this->Resolution);

    vtkSmartPointer<vtkTransform> y_rotation_120 = vtkSmartPointer<vtkTransform>::New();
    y_rotation_120->RotateWXYZ(120, 0, 1, 0);
    transformFilter->SetInputConnection(pad_sphere_2->GetOutputPort());
    transformFilter->SetTransform(y_rotation_120);
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> pad_sphere_2_rotated = vtkSmartPointer<vtkPolyData>::New();
    pad_sphere_2_rotated->ShallowCopy(transformFilter->GetOutput());

    vtkSmartPointer<vtkTransform> translation_towards_big_cyl = vtkSmartPointer<vtkTransform>::New();
    translation_towards_big_cyl->Translate(big_cyl_1->GetCenter()[0], big_cyl_1->GetCenter()[1], big_cyl_1->GetCenter()[2]);
    transformFilter->SetInputData(pad_sphere_2_rotated);
    transformFilter->SetTransform(translation_towards_big_cyl);
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> pad_sphere_2_final = vtkSmartPointer<vtkPolyData>::New();
    pad_sphere_2_final->ShallowCopy(transformFilter->GetOutput());

    // the third one needs to be rotated around the cylinder (240) 
    // apply the rotation then translate
    vtkSmartPointer<vtkSphereSource> pad_sphere_3 = vtkSmartPointer<vtkSphereSource>::New();
    pad_sphere_3->SetCenter(pad_sphere_1->GetCenter()[0], pad_sphere_1->GetCenter()[1], pad_sphere_1->GetCenter()[2]);
    pad_sphere_3->SetRadius(this->PadRadius);
    pad_sphere_3->SetThetaResolution(this->Resolution);
    pad_sphere_3->SetPhiResolution(this->Resolution);

    vtkSmartPointer<vtkTransform>  y_rotation_240 = vtkSmartPointer<vtkTransform>::New();
    y_rotation_240->RotateWXYZ(240, 0, 1, 0);
    transformFilter->SetTransform(y_rotation_240);
    transformFilter->SetInputConnection(pad_sphere_3->GetOutputPort());
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> pad_sphere_3_rotated = vtkSmartPointer<vtkPolyData>::New();
    pad_sphere_3_rotated->ShallowCopy(transformFilter->GetOutput());

    transformFilter->SetTransform(translation_towards_big_cyl);
    transformFilter->SetInputData(pad_sphere_3_rotated);
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> pad_sphere_3_final = vtkSmartPointer<vtkPolyData>::New();
    pad_sphere_3_final->ShallowCopy(transformFilter->GetOutput());

    transformFilter->SetInputConnection(pad_sphere_1->GetOutputPort());
    transformFilter->SetTransform(translation_towards_big_cyl);
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> pad_sphere_1_final = vtkSmartPointer<vtkPolyData>::New();
    pad_sphere_1_final->ShallowCopy(transformFilter->GetOutput());

    // connect the big cylinder to the pads to create one component
    vtkAppendPolyData * connected_component_1_final = vtkAppendPolyData::New();
    connected_component_1_final->AddInputData(pad_sphere_1_final);
    connected_component_1_final->AddInputData(pad_sphere_2_final);
    connected_component_1_final->AddInputData(pad_sphere_3_final);
    connected_component_1_final->AddInputData(big_cyl_1->GetOutput());
    connected_component_1_final->Update();

    // the cylinder that represent the engine
    vtkCylinderSource * engine_cyl = vtkCylinderSource::New();
    engine_cyl->SetRadius(this->EngineCylinderRadius);
    engine_cyl->SetHeight(this->EngineCylinderHeight);
    engine_cyl->SetResolution(this->Resolution);
    engine_cyl->SetCenter(big_cyl_1->GetCenter()[0], big_cyl_1->GetCenter()[1] + this->LargeCylinderHeight / 2 + this->EngineCylinderHeight / 2, big_cyl_1->GetCenter()[2]);

    // creating a half-sphere with a closed surface representing the drill
    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetCenter(big_cyl_1->GetCenter()[0], big_cyl_1->GetCenter()[1] + this->EngineCylinderHeight + this->LargeCylinderHeight / 2, big_cyl_1->GetCenter()[2]);
    sphere->SetRadius(this->HeadLength);
    sphere->SetThetaResolution(this->Resolution);
    sphere->SetPhiResolution(this->Resolution);
    vtkSmartPointer<vtkClipClosedSurface> sphere_final = vtkSmartPointer<vtkClipClosedSurface>::New();
    sphere_final->SetInputConnection(sphere->GetOutputPort());
    vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
    plane->SetOrigin(big_cyl_1->GetCenter()[0], big_cyl_1->GetCenter()[1] + this->LargeCylinderHeight / 2 + this->EngineCylinderHeight, big_cyl_1->GetCenter()[2]);
    plane->SetNormal(0, 1, 0);
    vtkSmartPointer<vtkPlaneCollection> plane_collection = vtkSmartPointer<vtkPlaneCollection>::New();
    plane_collection->AddItem(plane);
    sphere_final->SetClippingPlanes(plane_collection);

    // bellow cylinder
    int nPoints = this->BellowPointsN;								// this number decides how many points are used to represent the bellow
    double step = this->BellowCylinderHeight / (nPoints-1);			// the length between two points
    double step_angle = - (this->Angle / nPoints) * M_PI/180;		// the degree of the rotation
    double sum_y = -this->LargeCylinderHeight / 2;					// the y of each point (decremented in the following loop)

    vtkSmartPointer<vtkPoints> points_ = vtkSmartPointer<vtkPoints>::New();
    points_->SetNumberOfPoints(nPoints);

    // variant: update the point  in y, rotate, and then save the point
    for (int ii = 0; ii < nPoints; ++ii)
    {
      double x = 0;
      double sina = sin(step_angle*ii);
      double cosa = cos(step_angle*ii);
      double res_x;
      double res_y;

      // the ii-th point has to be rotated ii times the step_angle around z
      res_x = x * cosa + sum_y * sina;
      res_y = x * -sina + sum_y * cosa;
      points_->SetPoint(ii, res_x, res_y, 0);
      sum_y -= step;
    }

    // the line source will connect all the given points
    vtkSmartPointer<vtkLineSource> lineSource_ = vtkSmartPointer<vtkLineSource>::New();
    lineSource_->SetPoints(points_);
    lineSource_->Update();

    // this is where we define 2 radii to give the bellow its appearance
    vtkSmartPointer<vtkDoubleArray> radius_ = vtkSmartPointer<vtkDoubleArray>::New();
    radius_->SetName("TubeRadius");
    radius_->SetNumberOfTuples(nPoints);
    for (int ii = 0; ii < nPoints; ++ii)
    {
      if (ii % 2 == 0)
        radius_->SetTuple1(ii, this->BellowCylinderRadius);	// 1. Radius
      else
        radius_->SetTuple1(ii, this->BellowCylinderRadius / 2);	// 2. Radius
    }
    lineSource_->GetOutput()->GetPointData()->AddArray(radius_);
    lineSource_->GetOutput()->GetPointData()->SetActiveScalars(radius_->GetName());

    // tubefilter to put everything together
    vtkSmartPointer<vtkTubeFilter> tubeFilter_ = vtkSmartPointer<vtkTubeFilter>::New();
    tubeFilter_->SetInputData(lineSource_->GetOutput());
    tubeFilter_->CappingOn();
    tubeFilter_->SetNumberOfSides(this->Resolution);
    tubeFilter_->SetVaryRadiusToVaryRadiusByAbsoluteScalar();
    tubeFilter_->Update();

    // translating the bellow towards the correct position
    transformFilter->SetTransform(translation_towards_big_cyl);
    transformFilter->SetInputConnection(tubeFilter_->GetOutputPort());
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> connected_bellows_final = vtkSmartPointer<vtkPolyData>::New();
    connected_bellows_final->ShallowCopy(transformFilter->GetOutput());

    // create the second big cylinder (with pads) by copying the first one
    vtkSmartPointer<vtkPolyData> connected_component_2 = vtkSmartPointer<vtkPolyData>::New();
    connected_component_2->ShallowCopy(connected_component_1_final->GetOutput());

    // the second big cylinder translated back
    vtkSmartPointer<vtkTransform> translation_away_big_cyl = vtkSmartPointer<vtkTransform>::New();
    translation_away_big_cyl->Translate(-big_cyl_1->GetCenter()[0], -big_cyl_1->GetCenter()[1], -big_cyl_1->GetCenter()[2]);
    transformFilter->SetTransform(translation_away_big_cyl);
    transformFilter->SetInputData(connected_component_2);
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> connected_component_2_translated_1 = vtkSmartPointer<vtkPolyData>::New();
    connected_component_2_translated_1->ShallowCopy(transformFilter->GetOutput());

    // right now it overlaps with the first connected component -> move it behind the bellows
    vtkSmartPointer<vtkTransform> translation_behind_bellow = vtkSmartPointer<vtkTransform>::New();
    translation_behind_bellow->Translate(0, -this->BellowCylinderHeight - this->LargeCylinderHeight, 0);
    transformFilter->SetTransform(translation_behind_bellow);
    transformFilter->SetInputData(connected_component_2_translated_1);
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> connected_component_2_translated_2 = vtkSmartPointer<vtkPolyData>::New();
    connected_component_2_translated_2->ShallowCopy(transformFilter->GetOutput());

    // rotate y around 60 degree to match the pads
    vtkSmartPointer<vtkTransform>  y_rotation_60 = vtkSmartPointer<vtkTransform>::New();
    y_rotation_60->RotateWXYZ(60, 0, 1, 0);
    transformFilter->SetTransform(y_rotation_60);
    transformFilter->SetInputData(connected_component_2_translated_2);
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> connected_component_2_rotated_1 = vtkSmartPointer<vtkPolyData>::New();
    connected_component_2_rotated_1->ShallowCopy(transformFilter->GetOutput());

    // rotate z around given angle in case the robot is curved
    vtkSmartPointer<vtkTransform>  z_rotation = vtkSmartPointer<vtkTransform>::New();
    z_rotation->RotateWXYZ((this->Angle / nPoints)*(nPoints-1), 0, 0, 1); //
    transformFilter->SetTransform(z_rotation);
    transformFilter->SetInputData(connected_component_2_rotated_1);
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> connected_component_2_rotated_2 = vtkSmartPointer<vtkPolyData>::New();
    connected_component_2_rotated_2->ShallowCopy(transformFilter->GetOutput());

    // translate toward the correct position
    transformFilter->SetTransform(translation_towards_big_cyl);
    transformFilter->SetInputData(connected_component_2_rotated_2);
    transformFilter->Update();
    vtkSmartPointer<vtkPolyData> connected_component_2_final = vtkSmartPointer<vtkPolyData>::New();
    connected_component_2_final->ShallowCopy(transformFilter->GetOutput());

    // all components are connected using the vtkappendpolydata
    vtkAppendPolyData * appender = vtkAppendPolyData::New();
    appender->AddInputData(connected_bellows_final);
    appender->AddInputConnection(sphere_final->GetOutputPort());
    appender->AddInputConnection(engine_cyl->GetOutputPort());
    appender->AddInputData(connected_component_1_final->GetOutput());
    appender->AddInputData(connected_component_2_final);
    appender->Update();


    // this is where we color the final result
    vtkSmartPointer<vtkUnsignedCharArray> cellData = vtkSmartPointer<vtkUnsignedCharArray>::New();
    cellData->SetNumberOfComponents(3);
    cellData->SetNumberOfTuples(appender->GetOutput()->GetNumberOfCells());

    // default to cover all cases
    float rgb[3];
    rgb[0] = 255;
    rgb[1] = 255;
    rgb[2] = 255;

    // we color cellwise, which means we have to calculate the number of cells in each component
    int nHeadCells = sphere_final->GetOutput()->GetNumberOfCells();
    int nEngineCells = engine_cyl->GetOutput()->GetNumberOfCells();
    int nConnectedComponent = connected_component_1_final->GetOutput()->GetNumberOfCells();
    int nPadCells = pad_sphere_1_final->GetNumberOfCells();
    int nBigCylCells = big_cyl_1->GetOutput()->GetNumberOfCells();
    int nBellows = connected_bellows_final->GetNumberOfCells();

    // run through the MukRobotSource to color each component
    for (int i = 0; i < appender->GetOutput()->GetNumberOfCells(); i++)
    {
      if (i < nHeadCells)	// head
      {
        cellData->InsertTuple(i, this->HeadRGB);
      }
      else if (i < nHeadCells + nEngineCells)	// engine
      {
        cellData->InsertTuple(i, this->EngineRGB);
      }
      else if (i  <  nHeadCells + nEngineCells + 2 * nConnectedComponent)	// connectedComponents
      {
        if (i < nHeadCells + nEngineCells + nConnectedComponent)
        {
          // first connectedComponent
          if (i < nHeadCells + nEngineCells + 3 * nPadCells)
          {
            cellData->InsertTuple(i, this->PadRGB);
          }
          else
          {
            cellData->InsertTuple(i, this->LargeCylinderRGB);
          }
        }
        else
        {
          // second connectedComponent
          if (i < nHeadCells + nEngineCells + nConnectedComponent + 3 * nPadCells)
          {
            cellData->InsertTuple(i, this->PadRGB);
          }
          else
          {
            cellData->InsertTuple(i, this->LargeCylinderRGB);
          }
        }
      }
      else if (i < nHeadCells + nEngineCells + 2 * nConnectedComponent + nBellows)	//  bellow
      {
        cellData->InsertTuple(i, this->BellowCylinderRGB);
      }
      else
      {
        cellData->InsertTuple(i, rgb);
      }
    }
    appender->GetOutput()->GetCellData()->SetScalars(cellData);
    output->DeepCopy(appender->GetOutput());
    appender->Delete();
    return 1;
  }

  /**
  */
  int MukRobotSource::RequestInformation(vtkInformation *vtkNotUsed(request), vtkInformationVector **vtkNotUsed(inputVector), vtkInformationVector *outputVector)
  {
    vtkInformation *outInfo = outputVector->GetInformationObject(0);
    outInfo->Set(CAN_HANDLE_PIECE_REQUEST(), 1);
    return 1;
  }
}
}