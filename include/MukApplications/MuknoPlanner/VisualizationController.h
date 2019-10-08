#pragma once
#include "BaseController.h"

#include "MukCommon/INavigator.h"

#include "gstd/DynamicProperty.h"

#include <QWidget>
#include <QThread>

namespace gris
{
  namespace muk
  {
    class MukPath;
    class MukTransform;
    class VisAbstractObject;
    class VisCursor3D;

    /**
    */
    class VisualizationController : public BaseController
    {
      Q_OBJECT

      public:
        VisualizationController();
        virtual ~VisualizationController() = default;

      public:
        virtual void initialize() {}
        virtual void setupConnections();

      public:
        void addAbstractObject(std::shared_ptr<VisAbstractObject> pObj);
        void addTrajectory(const std::string key, int idx);
        void deleteAbstractObject(const std::string& name);
        void showBounds();
        void getVisWindow(int index);
        void defaultColorizeObstacles();

        VisCursor3D* requestCursor3D();
        void setCursorPosition(Vec3d pos);

        void showOnlyPath(const std::string& name, int idx);
        void showPaths();
        void hidePaths();
        void colorizePaths();

        // slots 
      public:
        void setRobotTransform(const MukTransform& transform);
        void setAdaptedPath(const MukPath& path);
        void setCurrentTargetOrientation(const Vec3d& state);
        // update navigation visualization
        void updateTransform(const MukTransform& transform);
        void updateNavigatorFeature(const INavigator::NavigatorFeature& feature, const bool& state);
        // slots for synchronization of NavigatorState with Visualization
        void updateNavState_Initialization(const bool pState);
        void updateNavState_Running(const bool pState);

       protected:
        void addTimer();
        void removeTimer();
        virtual void timerEvent(QTimerEvent* e);

      private:
        int mTimerId;
    };
  }
}
