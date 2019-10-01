/* Copyright (c) 2012, STANISLAW ADASZEWSKI
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of STANISLAW ADASZEWSKI nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL STANISLAW ADASZEWSKI BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
#include "private/muk.pch"
#include "AlgItemHandler.h"

#include "AlgConnection.h"
#include "AlgItem.h"
#include "AlgPort.h"

#include <QEvent>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsSceneMouseEvent>

namespace gris
{
namespace muk
{
  /**
  */
  AlgItemHandler::AlgItemHandler(QObject* parent) 
    : QObject(parent)
    , mpConn(nullptr)
  {
  }

  /** \brief install scene
  */
  void AlgItemHandler::install(QGraphicsScene* s)
  {
    s->installEventFilter(this);
    mpScene = s;
  }

  /** \brief Return the first user item at the given position

    Probably the first = the visible = the one drawn at last
  */
  QGraphicsItem* AlgItemHandler::itemAt(const QPointF& pos) const
  {
    QList<QGraphicsItem*> items = mpScene->items(QRectF(pos - QPointF(1, 1), QSize(3, 3)));
    foreach(QGraphicsItem* item, items)
    {
      if (item->type() > QGraphicsItem::UserType)
      {
        return item;
      }
    }
    return nullptr;
  }

  /** \brief delete the last added connection
  */
  void AlgItemHandler::deleteLastAddedConnection()
  {
    emit connectionDeleted(mpLastAddedConnection->getInputPort(), mpLastAddedConnection->getOutputPort());
    delete mpLastAddedConnection;
    mpLastAddedConnection = nullptr;
  }

  /** \brief event filter handles various actions:

    *	mouse pressed left:
     							port clicked: create new connection
     							block clicked: initiate drage and drop of the item
    *	mouse pressed right:
                  delete clicked object
    * mouse dopple clicked:
                  show input/output of the clicked port
    *	mouse moved:	
                  draw connection path
    *	mouse released:
                  set connection
  */
  bool AlgItemHandler::eventFilter(QObject* o, QEvent* e)
  {
    QGraphicsSceneMouseEvent* me = (QGraphicsSceneMouseEvent*)e;
    switch ((int)e->type())
    {
      case QEvent::GraphicsSceneMousePress:
      {
        switch ((int)me->button())
        {
          case Qt::LeftButton:
          {
            QGraphicsItem* item = itemAt(me->scenePos());
            if ( ! item)
            {
              emit backgroundClicked();
            }
            else if (item->type() == AlgPort::Type)
            {
              mpConn = new AlgConnection(nullptr);
              mpScene->addItem(mpConn);
              mpConn->setInputPort((AlgPort*)item);
              mpConn->setInputPos(item->scenePos());
              mpConn->setOutputPos(me->scenePos());
              mpConn->updatePath();
              return true;
            }
            else if (item->type() == AlgItem::Type)
            {
              auto* curItem = static_cast<AlgItem*>(item);
              emit blockClicked((curItem->getRefID()));
            }
            break;
          }
          case Qt::RightButton:
          {
            QGraphicsItem* item = itemAt(me->scenePos());
            if (item && (item->type() == AlgConnection::Type))
            {
              AlgConnection* curItem = static_cast<AlgConnection*>(item);
              emit connectionDeleted(curItem->getInputPort(), curItem->getOutputPort());
              delete item;
            }
            else if (item && (item->type() == AlgItem::Type))
            {
              AlgItem* curItem = static_cast<AlgItem*>(item);
              emit deleteBlock(curItem->getRefID());
              delete item;
            }
            break;
          }
        }
      }
      case QEvent::GraphicsSceneMouseDoubleClick:
      {
        QGraphicsItem* item = itemAt(me->scenePos());
        if (item && item->type() == AlgPort::Type)
        {
          AlgPort* curPort = static_cast<AlgPort*>(item);
          emit portClicked(curPort->getRefID(), curPort->isOutput(), curPort->getPortID());
        }
      }
      case QEvent::GraphicsSceneMouseMove:
      {
        if (mpConn)
        {
          mpConn->setOutputPos(me->scenePos());
          mpConn->updatePath();
          return true;
        }
        break;
      }
      case QEvent::GraphicsSceneMouseRelease:
      {
        if (mpConn && me->button() == Qt::LeftButton)
        {
          QGraphicsItem* item = itemAt(me->scenePos());
          if (item && item->type() == AlgPort::Type)
          {
            AlgPort* port1 = mpConn->getInputPort();
            AlgPort* port2 = (AlgPort*)item;

            if (port1->getAlgItem() != port2->getAlgItem() && port1->isOutput() != port2->isOutput() && !port1->isConnected(port2))
            {
              mpConn->setOutputPos(port2->scenePos());
              mpConn->setOutputPort(port2);
              mpConn->updatePath();
              mpLastAddedConnection = mpConn;
              mpConn = nullptr;
              emit connectionAdded(port1, port2);
              return true;
            }
          }

          delete mpConn;
          mpConn = nullptr;
          return true;
        }
        break;
      }
    }
    return QObject::eventFilter(o, e);
  }

  /**
  */
  int AlgItemHandler::algorithmAt(const QPointF& pos) const
  {
    auto* item = dynamic_cast<AlgItem*>(itemAt(pos));
    if (item)
    {
      return item->getRefID();
    }
    return -1;
  }
}
}