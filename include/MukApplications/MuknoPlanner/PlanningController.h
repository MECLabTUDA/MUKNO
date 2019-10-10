#pragma once
#include "BaseController.h"

QT_BEGIN_NAMESPACE
class QTreeWidgetItem;
QT_END_NAMESPACE

namespace gris
{
  namespace muk
  {
    /**
    */
    class PlanningController : public BaseController
    {
      Q_OBJECT


      public:
        virtual void initialize();
        void setupConnnections();

      signals:
        // when the planningChanged in a significant manner this is send to SelectionController
        void planningChanged();

      public:
        void changePathCollection(const std::string& key);
        void changePathIndex(int index);
        void addPathCollection(const std::string& name);
        void addReplanningPathCollection(const std::string& key, int pathIdx, int stateIdx);
        void deletePathCollection(const std::string& name);
        void clearPathCollection(const std::string& name);
        void selectPathCollection(const std::string& key);
        void updateProblemDefinition(const std::string& key);
        
        void deleteObstacle(const std::string& name);
        void changeSamplingMethod();
        // from scene widget
        void setDefaultBounds();
        void minimizeBounds();
        void addObjectBounds();

        // actual planning
        void createPaths(const std::string& pathCollection, size_t numNewPaths);
        void createPaths(const std::string& pathCollection, double availableTime);
        void updatePath(const std::string& name, int pathIdx);
        void updatePlanner();
        void updatePruner();
        void updateInterpolator();
        void setPlanner(const std::string& name);
        void setPruner(const std::string& name);
        void setInterpolator(const std::string& name);
        
        // GUi
        void initializeTabPlanning();

      private:
    };
  }
}
