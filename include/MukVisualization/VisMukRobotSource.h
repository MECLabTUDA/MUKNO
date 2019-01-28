#pragma once

#include "IVisRobot.h"
#include "MukRobotSource.h"

#include <vtkSmartPointer.h>
#include <vtkTransformPolyDataFilter.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API VisMukRobotSource : public IVisRobot
    {
      public:
        VisMukRobotSource();
        virtual ~VisMukRobotSource() {}

        static  const char* s_name()        { return "VisMukRobotSource"; }
        virtual const char* name()    const { return s_name(); };

      public:
        void                setScale(float d);
        float               getScale()         const { return mScale[0]; };

      public:
        virtual void setState(const MukState& state);
        virtual void setTransform(vtkTransform* pTransform);
        virtual void loadModel();

      private:
        float mScale[3];
        vtkSmartPointer<vtkTransform> mpRotation;     // rotates corresponding of MukState
        vtkSmartPointer<vtkTransform> mpTranslation;  // translates to position of MukState

        vtkSmartPointer<vtkTransform> mpToOrigin;     // helper transform until MukRobotSource has the right local coordinate frame
        vtkSmartPointer<vtkTransform> mpScale;        // scales the stl file

        vtkSmartPointer<MukRobotSource> mpRobot;   // transforms from stl-file to robot coordinate system
        vtkSmartPointer<vtkTransform> mpRobotToWorld; // transforms from robot to world coordinate system

        vtkSmartPointer<vtkTransform> mpConcatenatedTransform;
        vtkSmartPointer<vtkTransformPolyDataFilter> mpTransformFilter;
    };
  }
}
