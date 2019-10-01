#include "private/muk.pch"
#include "SceneWidget.h"
#include "MukQRightClickMenu.h"

#include "MukCommon/MukException.h"

#include <QTreeWidgetItem>
#include <QMouseEvent>
#include <QDropEvent>
#include <QMimeData>

namespace
{
  QTreeWidgetItem* getTopLevelItem(QTreeWidget* pWidget, const std::string& str);
  QTreeWidgetItem* findChild(QTreeWidgetItem* pItem, const std::string& key);
  QTreeWidgetItem* findOrMakeChild(QTreeWidgetItem* pItem, const std::string& key);
  QColor toQtColor(const gris::Vec3d& color);
}

namespace gris
{
  namespace muk
  {
    /**
    */
    SceneWidget::SceneWidget(QWidget* parent)
      : QTreeWidget(parent)
    {
      headerItem()->setText(0, "Scene Components:");
      setAcceptDrops(true);
    }

    /**
    */
    SceneWidget::~SceneWidget()
    {
    }

    /**
    */
    void SceneWidget::setTopLevelItem(size_t index, const std::string& name)
    {
      auto* pItem = topLevelItem(static_cast<int>(index));
      pItem->setText(0, name.c_str());
    }

    /**
    */
    void SceneWidget::setItem(const std::vector<std::string>& strings)
    {
      auto* pItem = getTopLevelItem(this, strings.front());
      for (size_t i(1); i<strings.size(); ++i)
      {
        pItem = findOrMakeChild(pItem, strings[i]);
      }
    }

    /**
    */
    void SceneWidget::removeItem(const std::vector<std::string>& strings)
    {
      auto* pItem = getTopLevelItem(this, strings.front());
      if (strings.size() > 0)
      {
        for (size_t i(1); i<strings.size()-1; ++i)
        {
          pItem = ::findChild(pItem, strings[i]);
        }
      }
      auto* pChild = ::findChild(pItem, strings.back());
      if (nullptr == pChild)
      {
        std::string info;
        for (size_t i(0); i<strings.size(); ++i)
          info += " " + strings[i];
        throw MUK_EXCEPTION("Item does not exist", info.c_str());
      }
      pItem->removeChild(pChild);
    }

    /**
    */
    void SceneWidget::setColor(const std::vector<std::string>& strings, const Vec3d& color)
    {
      auto* pItem = getTopLevelItem(this, strings.front());
      if (strings.size() > 0)
      {
        for (size_t i(1); i<strings.size(); ++i)
        {
          pItem = ::findChild(pItem, strings[i]);
          if (nullptr == pItem)
          {
            std::string info;
            for (size_t i(0); i<strings.size(); ++i)
              info += " " + strings[i];
            throw MUK_EXCEPTION("Item does not exist", info.c_str());
          }
        }
      }
      pItem->setBackgroundColor(0, toQtColor(color));
    }

    /**
    */
    void SceneWidget::dragMoveEvent(QDragMoveEvent* event)
    {
      event->acceptProposedAction();
    }

    /**
    */
    void SceneWidget::dragEnterEvent(QDragEnterEvent* event)
    {
      event->acceptProposedAction();
    }

    /** \brief
    */
    void SceneWidget::dropEvent(QDropEvent* event)
    {
      const auto* mimeData = event->mimeData();
      if (mimeData->hasText()) 
      {
        const auto str = mimeData->text().toStdString();
        emit textDropped(str);
        event->acceptProposedAction();
      }
    }

    /**
    */
    void SceneWidget::mousePressEvent(QMouseEvent* event)
    {
      QTreeWidget::mousePressEvent(event);
      if (event->button() == Qt::RightButton)
      {
        MukQRightClickMenu menu;
        QPoint pos = QCursor::pos();
        menu.setPosition(pos.x(), pos.y());        
        menu.addHandle("Save Obstacle",      [this] () { emit this->saveObjectClicked(); });
        menu.addHandle("Delete Obstacle",    [this] () { emit this->deleteObstacleClicked(); });
        menu.addHandle("Delete Object",      [this] () { emit this->deleteAbstractObjectClicked(); });
        menu.addSeparator();
        const auto* submenu = "Collection";
        menu.addHandle(submenu, "Hide",      [this] () { emit this->hidePaths(); });
        menu.addHandle(submenu, "Show",      [this] () { emit this->showPaths(); });
        menu.addHandle(submenu, "colorize",  [this] () { emit this->colorize(); });
        menu.addHandle("Show Trajectory",    [this] () { emit this->showOrientationPathClicked(); });
        menu.addHandle("Show Planner graph", [this] () { emit this->showPlannerGraphClicked(); });
        menu.addSeparator();
        menu.addHandle("Default colorize Obstacles", [this] () { emit this->defaultColorizeObstaclesClicked(); });
        menu.addSeparator();
        menu.addHandle("Show Bounds",        [this] () { emit this->showBoundsClicked(); });
        menu.addHandle("Set Default Bounds", [this] () { emit this->setDefaultBoundsClicked(); });
        menu.addHandle("Add Object Bounds",  [this] () { emit this->addObjectBoundsClicked(); });
        menu.addHandle("Minimize Bounds",    [this] () { emit this->minimizeBoundsClicked(); });
        menu.exec();
      }    
    }
  }
}


namespace
{
  /**
  */
  QTreeWidgetItem* getTopLevelItem(QTreeWidget* pWidget, const std::string& str)
  {
    for (int i(0); i<pWidget->topLevelItemCount(); ++i)
    {
      if (str == pWidget->topLevelItem(i)->text(0).toLocal8Bit().constData())
      {
        return pWidget->topLevelItem(i);
      }
    }
    throw MUK_EXCEPTION("TopLevelItem does not exist", str.c_str());
  }

  /**
  */
  QTreeWidgetItem* findOrMakeChild(QTreeWidgetItem* pItem, const std::string& key)
  {
    for (int i(0); i<pItem->childCount(); ++i)
    {
      if (key == pItem->child(i)->text(0).toLocal8Bit().constData())
        return pItem->child(i);
    }
    auto* pChild = new QTreeWidgetItem();
    pChild->setText(0, key.c_str());
    pItem->addChild(pChild);
    return pChild;
  }

  /**
  */
  QTreeWidgetItem* findChild(QTreeWidgetItem* pItem, const std::string& key)
  {
    for (int i(0); i<pItem->childCount(); ++i)
    {
      if (key == pItem->child(i)->text(0).toLocal8Bit().constData())
        return pItem->child(i);
    }
    return nullptr;
  }

  /**
  */
  QColor toQtColor(const gris::Vec3d& color)
  {
    return QColor(255*color.x(), 255*color.y(), 255*color.z());
  }
}