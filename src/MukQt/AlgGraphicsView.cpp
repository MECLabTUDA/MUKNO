#include "private/muk.pch"
#include "AlgGraphicsView.h"

#include <qmimedata.h>

namespace gris
{
namespace muk
{
  /**
  */
  AlgGraphicsView::AlgGraphicsView(QWidget* parent) 
    : QGraphicsView(parent)
  {
    setAcceptDrops(true);
  }

  /**
  */
  AlgGraphicsView::AlgGraphicsView(QGraphicsScene* scene, QWidget* parent)
    : QGraphicsView(scene, parent)
  {
    setAcceptDrops(true);
  }

  /** \brief Handles dropped items.

     Items can come from the QListWidget or from outside the GUI
  */
  void AlgGraphicsView::dropEvent(QDropEvent* event)
  {
    QListWidget* listWidget = (QListWidget*)event->source();
    if (listWidget != nullptr)
    {
      auto pos = mapToScene(event->pos());
      emit filterDropped(listWidget->selectedItems()[0]->text(), pos);
      event->acceptProposedAction();
      return;
    }
    const auto* mimeData = event->mimeData();
    if (mimeData->hasText()) 
    {
      auto pos = mapToScene(event->pos());
      emit textDropped(mimeData->text(), pos);
      //event->acceptProposedAction();
    }
  }

  /** \brief handle drag actions
  */
  void AlgGraphicsView::dragMoveEvent(QDragMoveEvent* event)
  {
    event->acceptProposedAction();
  }

  /** \brief handle drag enter events
  */
  void AlgGraphicsView::dragEnterEvent(QDragEnterEvent* event)
  {
    event->acceptProposedAction();
  }

  /** \brief allow zooming

    todo: check, if not zoomed in/out too far.
  */
  void AlgGraphicsView::wheelEvent(QWheelEvent *event)
  {
    // Typical Calculations (Ref Qt Doc)
    const auto delta = event->delta();
    if (delta < 0)
      scale(0.9, 0.9);
    else
      scale(1.1, 1.1);
  }
}
}