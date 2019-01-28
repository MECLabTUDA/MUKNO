#include "private/muk.pch"
#include "muk_qt_tools.h"
#include "gris_math.h"

#include <qtreewidget.h>

namespace gris
{
namespace muk
{
  /**
  */
  QTreeWidgetItem* findChild(const QTreeWidgetItem* pItem, const std::string& text, int column)
  {
    QTreeWidgetItem* result(nullptr);
    if (pItem == nullptr)
      return result;
    const auto N = pItem->childCount();
    for (int i(0); i<N; ++i)
    {
      if (text == pItem->child(i)->text(column).toLocal8Bit().constData())
      {
        result = pItem->child(i);
        break;
      }
    }
    return result;
  }
}
}