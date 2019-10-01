#pragma once

#include "muk_qt_api.h"

#include <QMenu>

#include <functional>

namespace gris
{
  namespace muk
  {
    class MUK_QT_API MukQRightClickMenu : public QMenu
    {
      public:
        explicit MukQRightClickMenu(QWidget* parent=0);
        virtual ~MukQRightClickMenu();

      public:      
        virtual void setPosition(double x, double y) { this->move(x, y); }

      public:
        typedef std::function<void()> Callback;
        void addHandle(const char* actionName, Callback cbHandle);
		    void addHandle(const char* actionName, Callback cbHandle, bool setCheckable);
        void addHandle(const char* subMenu, const char* actionName, Callback cbHandle);
    };


  }
}
