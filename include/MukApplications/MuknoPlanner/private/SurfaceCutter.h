#pragma once

#include "private/IMukInteraction.h"

#include "MukVisualization/VisAbstractObject.h"

#include <memory>

class vtkClipPolyData;
class vtkPolyData;
class vtkSphere;

namespace gris
{
  namespace muk
  {
    class VisCursor3D;
    class VisAbstractObject;

    /**
    */
    class SurfaceCutter : public IMukInteraction
    {
      public:
        static SurfaceCutter* New();
        vtkTypeMacro(SurfaceCutter, IMukInteraction);

      public:
        SurfaceCutter();
        ~SurfaceCutter();

      public:
        virtual void OnLeftButtonDown();
        virtual void OnMouseMove();
        virtual void OnRightButtonDown();
        virtual void OnRightButtonUp();
        virtual void OnMouseWheelForward();
        virtual void OnMouseWheelBackward();
        virtual void OnKeyPress();

        virtual void initialize();

      private:
        void accept();
        void discard();

      private:      
        void cutRegion();
        void finishInteraction();
        void pick();
        std::string getNewName();
        void adaptPickedObstacle();

      private:
        VisCursor3D* mpCursor;
        std::shared_ptr<VisAbstractObject> mpNewRegion;
        
        double mRadius;
        double mRadiusStepSize;
        
        vtkPolyData* mpPickedData;
        vtkSmartPointer<vtkClipPolyData> mpFilter;
        vtkSmartPointer<vtkSphere>       mpClipFunction;
    };
  }
}
