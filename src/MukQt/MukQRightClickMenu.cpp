#include "private/muk.pch"
#include "MukQRightClickMenu.h"
#include <qsignalmapper.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    MukQRightClickMenu::MukQRightClickMenu(QWidget* parent)
      : QMenu(parent)
    {
        QPoint p = QCursor::pos();
        QRect geo = this->geometry();
        this->move(p.x()+geo.width() - this->width(), p.y());
    }

    /**
    */
    MukQRightClickMenu::~MukQRightClickMenu()
    {
    }
    
    /**
    */
    void MukQRightClickMenu::addHandle(const char* actionName, Callback cbHandle)
    {
      auto action = this->addAction(actionName);
      connect(action, &QAction::triggered, cbHandle);
    }

    void MukQRightClickMenu::addHandle(const char* actionName, Callback cbHandle, bool _setCheckable)
    {
	    auto action = this->addAction(actionName);
	    action->setCheckable(_setCheckable);
	    action->setChecked(true);
	    connect(action, &QAction::triggered, cbHandle);
    }

    void MukQRightClickMenu::addHandle(const char* subMenu, const char* actionName, Callback cbHandle)
    {
      auto v = children();
      QMenu* menu(nullptr);
      for(auto* c : v)
      {
        auto pMenu = dynamic_cast<QMenu*>(c);
        if(pMenu)
        {
          menu = pMenu;
          break;
        }
      }
      if (menu == nullptr)
        menu = addMenu(subMenu);
      auto action = menu->addAction(actionName);
      connect(action, &QAction::triggered, cbHandle);
    }


  }
}
