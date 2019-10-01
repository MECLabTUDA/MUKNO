#include "private/muk.pch"
#include "AlgConnection.h"
#include "AlgItem.h"
#include "AlgPort.h"

#include <QFontMetrics>
#include <QPen>
#include <QtWidgets/QGraphicsScene>

namespace gris
{
namespace muk
{
  /** \brief create new port
  */
  AlgPort::AlgPort(bool isOutput, QGraphicsItem* parent)
    : QGraphicsPathItem(parent)
    , mPortRadius(5)
    , mIsOutput(isOutput)
{
    mPortColor.setRgb(0,255,0);
    mPortFrameColor.setRgb(0, 0, 0);
    QPainterPath p;
    p.addEllipse(-mPortRadius, -mPortRadius, 2*mPortRadius, 2*mPortRadius);
    setPath(p);
    setPen(mPortFrameColor);
    setBrush(mPortColor);
    setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
  }

  /**
  */
  int AlgPort::getRefID() const 
  {
    return mpAlgItem->getRefID(); 
  };

  /**
  */
  AlgPort::~AlgPort()
  {
    foreach(AlgConnection* conn, mConnections)
      delete conn;
  }
  
  /** \brief return true if port is connected else false
  */
  bool AlgPort::isConnected(AlgPort* other) const
  {
    foreach(AlgConnection* conn, mConnections)
    {
      if (conn->getInputPort() == other || conn->getOutputPort() == other)
        return true;
    }
    return false;
  }

  /** 

    \return QVariant if item has changed
  */
  QVariant AlgPort::itemChange(GraphicsItemChange change, const QVariant& value)
  {
    if (change == ItemScenePositionHasChanged)
    {
      foreach(AlgConnection* conn, mConnections)
      {
        conn->updatePosFromPorts();
        conn->updatePath();
      }
    }
    return value;
  }
}
}