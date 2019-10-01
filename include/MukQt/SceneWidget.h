#pragma once

#include "muk_qt_api.h"

#include "MukCommon/MukVector.h"

#include <qtreewidget.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_QT_API SceneWidget : public QTreeWidget
    {
      Q_OBJECT

      public:
        SceneWidget(QWidget* parent = nullptr);
        virtual ~SceneWidget();

      signals:
        void textDropped(const std::string& text);
        void requestProperty(QTreeWidgetItem* item, int column);

        void saveObjectClicked();
        void deleteObstacleClicked();
        void deleteAbstractObjectClicked();

        void hidePaths();
        void showPaths();
        void colorize();
        void showPlannerGraphClicked();
        void showOrientationPathClicked();

        void defaultColorizeObstaclesClicked();

        void showBoundsClicked();
        void setDefaultBoundsClicked();
        void addObjectBoundsClicked();
        void minimizeBoundsClicked();

      public:
        void setTopLevelItem(size_t index, const std::string& name);
        void setItem(const std::vector<std::string>& strings);
        void removeItem(const std::vector<std::string>& strings);
        void setColor(const std::vector<std::string>& strings, const Vec3d& color);
                
      protected:
        virtual void mousePressEvent(QMouseEvent *);
        virtual void dropEvent(QDropEvent* event);
        virtual void dragMoveEvent(QDragMoveEvent* event);
        virtual void dragEnterEvent(QDragEnterEvent* event);
    };
  }
}

