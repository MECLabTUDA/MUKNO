#include "private/muk.pch"
#include "VisMukRobotSource.h"

#include "MukCommon/MukState.h"
#include "MukCommon/mukIO.h"
#include "MukCommon/MukException.h"
//#include "MukCommon/geometry.h"

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#define _USE_MATH_DEFINES
#include <math.h> 

namespace
{  
  using gris::Vec3d;
  using namespace gris::muk;
  /**
  */
  void rotateOntoVector(const Vec3d& from_, const Vec3d& to_, double& angle, Vec3d& axis)
  {
    Vec3d from(from_);
    from.normalize();
    Vec3d to(to_);
    to.normalize();

    axis = from.cross(to);
    axis.normalize();    

    double const dotP = from.dot(to);
    angle = acos( abs(dotP) );
    if (abs(angle) < 10e-5)
    {
      angle = 0; //M_PI - angle;
    }
    if (dotP < 0)
      angle = M_PI -angle;
  }  
}

namespace gris
{
  namespace muk
  {
    /** \brief A visualization of the MUKNO drilling robot prototype

       Orientation is adjusted tpo the navigation library's convention that the x-axes of the local coordinate frame is the line of view (i.e. the tangent vector of a MukState)
    */
    VisMukRobotSource::VisMukRobotSource()
    {
      std::fill(mScale, mScale+3, 1.0f);
      setName(name());

      declareProperty<float>("Scale", 
        [&] (float d) { this->setScale(d); },
        [&] ()        { return this->getScale(); });

      mpRobot       = make_vtk<MukRobotSource>();
      mpRobot->SetPadRGB(0, 204, 255);
      mpRobot->SetPadRadius(0.5);
      mpRobot->SetLargeCylinderRGB(204, 0 ,0);

      mpToOrigin    = make_vtk<vtkTransform>();
      mpToOrigin->RotateY(90); // because MukRobotSource is defined that way atm
      mpToOrigin->RotateX(90);
      mpScale       = make_vtk<vtkTransform>();
      mpScale->Scale(mScale);
      mpRobotToWorld = make_vtk<vtkTransform>();

      mpConcatenatedTransform = make_vtk<vtkTransform>();
      mpConcatenatedTransform->PostMultiply();
      mpConcatenatedTransform->Concatenate(mpToOrigin);
      mpConcatenatedTransform->Concatenate(mpScale);
      mpConcatenatedTransform->Concatenate(mpRobotToWorld);
      mpTransformFilter = make_vtk<vtkTransformPolyDataFilter>();    
      mpTransformFilter->SetTransform(mpConcatenatedTransform);
      mpMapper->SetInputConnection(mpTransformFilter->GetOutputPort());
      try
      {
        loadModel();
      }
      catch (std::exception& e)
      {
        LOG_LINE << "==== WARNING: Failed to update robot source! ====";
        LOG_LINE << "error: " << e.what();
      }
    }

    /**
    */
    void VisMukRobotSource::setScale(float d)
    {
      const double reset = 1/mScale[0];
      mpScale->Scale(reset, reset, reset);
      std::fill(mScale, mScale+3, d);
      mpScale->Scale(mScale);
      mpScale->Update();
    }

    /**
    */
    void VisMukRobotSource::loadModel()
    {
      mpRobot->Update();
      mpTransformFilter->SetInputData(mpRobot->GetOutput());
      mpTransformFilter->Update();
      mpMapper->Update();
    }

    /**
    */
    void VisMukRobotSource::setState(const MukState& state)
    {
      //double angle(0);
      //Vec3d mukAxis(0,0,0);
      //// from STL coordinate system
      //// to state direction
      //rotateOntoVector(Vec3d(0,0,1), state.tangent, angle, mukAxis);
      //float axis[3] = { static_cast<float>(mukAxis.x()), static_cast<float>(mukAxis.y()), static_cast<float>(mukAxis.z()) };
      //angle = angle * 180 / M_PI;

      //mpRotation->Identity();
      //mpRotation->RotateWXYZ(angle, axis);
      //mpTranslation->Identity();
      //mpTranslation->Translate(state.coords.x(), state.coords.y(), state.coords.z());
      mpConcatenatedTransform->Modified();
    }

    /**
    */
    void VisMukRobotSource::setTransform(vtkTransform* pTransform)
    {
      mpRobotToWorld->DeepCopy(pTransform);
      mpConcatenatedTransform->Update();
    }
  }
}