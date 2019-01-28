#pragma once

#include "IVisRobot.h"

class vtkTransform;
class vtkTransformPolyDataFilter;

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API VisPreliminaryTestRobot : public IVisRobot
    {
      public:
        VisPreliminaryTestRobot();
        virtual ~VisPreliminaryTestRobot() {}

        static  const char* s_name()        { return "VisPreliminaryTestRobot"; }
        virtual const char* name()    const { return s_name(); };
        
      public:
        void                setFileSource(const std::string& filename);
        const std::string&  getFileSource()                             const { return mFileSource; };
        void                setScale(float d);
        float               getScale()                                  const { return mScale[0]; };
        
      public:
        virtual void setState(const MukState& state);
        virtual void setTransform(vtkTransform* pTransform);
        void         loadModel() { setFileSource(mFileSource); }

      private:
        float mScale[3];
        std::string mFileSource;        
        vtkSmartPointer<vtkTransform> mpToOrigin;
        vtkSmartPointer<vtkTransform> mpRotation;     // rotates corresponding of MukState
        vtkSmartPointer<vtkTransform> mpTranslation;  // translates to position of MukState
        vtkSmartPointer<vtkTransform> mpScale;        // scales the stl file

        vtkSmartPointer<vtkTransform> mpStlToRobot;   // transforms from stl-file to robot coordinate system
        vtkSmartPointer<vtkTransform> mpRobotToWorld; // transforms from robot to world coordinate system
        
        vtkSmartPointer<vtkTransform> mpConcatenatedTransform;
        vtkSmartPointer<vtkTransformPolyDataFilter> mpTransformFilter;
    };
  }
}
