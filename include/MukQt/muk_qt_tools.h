#pragma once

#include "muk_qt_api.h"

#include <qnamespace.h>

QT_BEGIN_NAMESPACE
class QTreeWidgetItem;
QT_END_NAMESPACE

namespace gris
{
  namespace muk
  {
    template<typename... Args> 
    struct SELECT 
    { 
      template<typename C, typename R = void> 
      static constexpr auto OVERLOAD_OF( R (C::*pmf)(Args...) ) -> decltype(pmf) 
      { 
        return pmf;
      } 
    };

    MUK_QT_API QTreeWidgetItem* findChild(const QTreeWidgetItem* pItem, const std::string& text, int column = 0);
  }
}
