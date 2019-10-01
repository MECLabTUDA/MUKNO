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
#include "AlgConnection.h"

#include "AlgPort.h"

#include <QBrush>
#include <QPen>
#include <QtWidgets/QGraphicsScene>

namespace gris
{
namespace muk
{

  AlgConnection::AlgConnection(QGraphicsItem* parent)
    : QGraphicsPathItem(parent)
    , mInputPort (0)
    , mOutputPort(0)
  {
    setPen(QPen(QColor("#111111"), 4));
    setBrush(Qt::NoBrush);
    setZValue(-1);
  }

  /**
  */
  AlgConnection::~AlgConnection()
  {
    if (mInputPort)
      mInputPort->getConnections().remove(mInputPort->getConnections().indexOf(this));
    if (mOutputPort)
      mOutputPort->getConnections().remove(mOutputPort->getConnections().indexOf(this));
  }

  /**
  */
  void AlgConnection::setInputPort(AlgPort* p)
  {
    mInputPort = p;
    mInputPort->getConnections().append(this);
  }

  /**
  */
  void AlgConnection::setOutputPort(AlgPort* p)
  {
    mOutputPort = p;
    mOutputPort->getConnections().append(this);
  }

  /**
  */
  void AlgConnection::updatePosFromPorts()
  {
    mInputPos  = mInputPort->scenePos();
    mOutputPos = mOutputPort->scenePos();
  }

  /**
  */
  void AlgConnection::updatePath()
  {
    QPainterPath p;
    p.moveTo(mInputPos);
    qreal dx = mOutputPos.x() - mInputPos.x();
    qreal dy = mOutputPos.y() - mInputPos.y();
    QPointF ctr1(mInputPos.x() + dx * 0.25, mInputPos.y() + dy * 0.1);
    QPointF ctr2(mInputPos.x() + dx * 0.75, mInputPos.y() + dy * 0.9);
    p.cubicTo(ctr1, ctr2, mOutputPos);
    setPath(p);
  }
}
}