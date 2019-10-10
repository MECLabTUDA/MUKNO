#pragma once

#include "BaseController.h"

#include "gstd/DynamicProperty.h"

QT_BEGIN_NAMESPACE
class QTreeWidgetItem;
QT_END_NAMESPACE

namespace gris
{
  namespace muk
  {
    class SceneWidget;
    class PropertyWidget;
    class INavigationContainer;
    class ProtectedNavigator;
    struct AuxProperty;
    struct AuxPropertyTuples;
    enum EnPropertyType;

    /**
    */
    class PropertyController : public BaseController
    {
      Q_OBJECT

        signals :
          // when the activ property changes (for obstacles) this signal is send to SelectionController
          void activChanged();

      public:
        PropertyController();
        ~PropertyController() = default;

      public:
        virtual void initialize();
        virtual void setupConnections();

      public:
        // property widget
        void showProperty(QTreeWidgetItem* pItem, int column);
        void showProperty(EnPropertyType type, gstd::DynamicProperty* pObj);
        void reloadProperty();
        void updateProperty();

        // drop event
        void evaluateDroppedText(const std::string& text);
        
        // scene widget
        void addAlgorithmOutput(const std::string& name);
        void deleteAlgorithmOutput(const std::string& name);
        void addAbstractObject(const std::string& name);
        void deleteAbstractObject(const std::string& name);
        void addPathCollection(const std::string& name);
        void deletePathCollection(const std::string& name);
        void setObstacle(const std::string& name);
        void deleteObstacle(const std::string& name);
        void setPlanner(const std::string& name);
        void setPruner(const std::string& name);
        void setInterpolator(const std::string& name);
        void updateNavigator();
        
        // request from scenewidget's right click menu
        void saveObject() const;
        void deleteObstacle();
        void deleteAbstractObject();
        void showTrajectory();
        void extractPlanningGraph();
        void showBounds();

        // other
        void setNavigation(const INavigationContainer* const pNavigator);
        void setNavigator(ProtectedNavigator* pObj);

      public:
        void initSceneWidget();

      private:
        void appendToWidget(const AuxPropertyTuples& auxtuples);
        
      private:
        SceneWidget* mpSceneWidget;
        PropertyWidget* mpPropertyWidget;
        std::vector<AuxProperty> mCachedProperties;
        EnPropertyType mCachedType;
        std::map<std::string, std::vector<std::string>> mStringSelections;
    };
  }
}