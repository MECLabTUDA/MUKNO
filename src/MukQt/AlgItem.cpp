#include "private/muk.pch"
#include "AlgItem.h"
#include "AlgPort.h"

#include <QFontMetrics>
#include <QPainter>
#include <QPen>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QStyleOptionGraphicsItem>

namespace gris
{
namespace muk
{

  /** \brief create new algorithm item to draw an AlgGraphicsView
  */
  AlgItem::AlgItem(QGraphicsItem* parent)
    : QGraphicsPathItem(parent)
  {
    setFlag(QGraphicsItem::ItemIsMovable);
    setFlag(QGraphicsItem::ItemIsSelectable);
    mHorizontalMargin = 50;
    mVerticalMargin   = 5;
    mVerticalNameMargin = 2.5;
    mPortMargin       = 5;
    mWidth  = mHorizontalMargin;
    mHeight = mVerticalMargin;

    mLabelName  = new QGraphicsTextItem(this);
    mLabelAlias = new QGraphicsTextItem(this);

    // check web safe colors
    const int cyan2[3] = { 51, 204, 255 };
    const int turquoise2[3] = { 0, 229, 238 };
    const int tan1[3] = { 255, 165, 79 };
    mBoxColor.setRgb(turquoise2[0], turquoise2[1], turquoise2[2]);
    mBoxColorSelected.setRgb(tan1[0], tan1[1], tan1[2]);
    mBoxFrameColor.setRgb(0,0,0);
  }

  /** \brief Add a new port to the AlgItem
  */
  void AlgItem::addPort(bool isOutput, int ptr)
  {
    AlgPort* port = new AlgPort(isOutput, this);
    port->setAlgItem(this);
    port->setPtr(ptr);
    if (isOutput)
    {
      port->setPortID((int)mOutputPorts.size());
      mOutputPorts.push_back(port);
    }
    else
    {
      port->setPortID((int)mInputPorts.size());
      mInputPorts.push_back(port);
    }
  }

  /** \brief ajusts the box size
    
     -------o------o-----
    |                    |
    |    mAlgorithmName  |
    |       mAlias       |
    |                    |
     ----------o---------

    vertical margin   = minimum x-dist between a label and frame
    horizontal margin = minimum y-dist between a label and frame
  */
  void AlgItem::rebuild()
  {
    QFontMetrics fm(scene()->font());
    // determine box width
    auto widthNames  = static_cast<float>(std::max(fm.width(mAlgorithmName), fm.width(mAlias)));
    auto widthPortsIn = mInputPorts.size() * mPortMargin;
    for (const auto& p : mInputPorts)
      widthPortsIn += 2 * p->getPortRadius();
    auto widthPortsOut = mOutputPorts.size() * mPortMargin;
    for (const auto& p : mOutputPorts)
      widthPortsOut += 2 * p->getPortRadius();
    auto widthPorts = std::max(widthPortsIn, widthPortsOut);
    auto boxWidth   = std::max(widthPorts, widthNames) + 2 * mHorizontalMargin;
    // determine box height
    float boxHeight = 2*fm.height() + 2*mVerticalMargin + mVerticalNameMargin;
    if (!mAlias.isEmpty())
      boxHeight += fm.height();
    // paint box
    QPainterPath p;
    // top left corner + width and height + radius of rounded corners
    p.addRoundedRect(-boxWidth / 2.0, -boxHeight / 2.0, boxWidth, boxHeight, 5, 5);
    setPath(p);
    // paint names
    auto stepY = fm.height();
    auto baseY  = - boxHeight / 2.0 + mVerticalMargin;
    mLabelName ->setPlainText(mAlgorithmName);
    mLabelAlias->setPlainText(mAlias);
    auto posX = -0.5f * fm.width(mAlgorithmName);
    auto posY = mAlias.isEmpty() ? baseY + 0.5f * fm.height() : baseY;
    mLabelName->setPos(posX, posY);
    posX      = -0.5f * fm.width(mAlias);
    mLabelAlias->setPos(posX, baseY + stepY + mVerticalNameMargin);
    // paint input ports
    auto stepX = boxWidth / (mInputPorts.size()+1);
    posX       = -boxWidth / 2.0;
    for (const auto& p : mInputPorts)
    {
      posX += stepX;
      p->setPos(posX, -boxHeight / 2);
    }
    // paint output ports
    stepX = boxWidth / (mOutputPorts.size()+1);
    posX  = -boxWidth / 2.0;
    for (const auto& p : mOutputPorts)
    {
      posX += stepX;
      p->setPos(posX, boxHeight / 2);
    }
  }

  /** \brief add new input port to the algorithm item
  */
  void AlgItem::addInputPort()
  {
    addPort(false);
  }

  /** \brief add new output port to the algorithm item
  */
  void AlgItem::addOutputPort()
  {
    addPort(true);
  }

  /** \brief add new list of input ports to the algorithm item
  */
  void AlgItem::addInputPorts(size_t n)
  {
    for (size_t i(0); i<n; ++i)
      addPort(false);
  }

  /** \brief add new list of output ports to the algorithm item
  */
  void AlgItem::addOutputPorts(size_t n)
  {
    for (size_t i(0); i<n; ++i)
      addPort(true);
  }

  /** \brief draw algorithm item on AlgGraphicsView
  */
  void AlgItem::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*)
  {
    if (isSelected()) 
    {
      painter->setBrush(mBoxColorSelected);
      painter->setPen(mBoxFrameColor);
    }
    else 
    {
      painter->setBrush(mBoxColor);
      painter->setPen(mBoxFrameColor);
    }
    painter->drawPath(path());
  }


  /** \brief clone algorithm item, with all of its ports
  */
  AlgItem* AlgItem::clone() const
  {
    auto* b = new AlgItem(nullptr);
    this->scene()->addItem(b);
    b->setName(mAlgorithmName.toStdString());
    for(auto* p : mInputPorts)
    {
      b->addPort(false, p->getPtr());
    }
    for (auto* p : mOutputPorts)
    {
      b->addPort(true, p->getPtr());
    }
    return b;
  }

  /** \return vector over all ports

   obviously, input and output is mixed. Caller has to search and count to reach an an input/output port
  */
  QVector<AlgPort*> AlgItem::getPorts() const
  {
    auto res = getInputPorts();
    auto out = getInputPorts();
    for (auto* p : out)
      res.append(p);
    return res;
  }

  /** \brief returns all output ports
  */
  QVector<AlgPort*> AlgItem::getOutputPorts() const
  {
    QVector<AlgPort*> res;
    for(auto* p : mOutputPorts)
    {
      res.append(p);
    }
    return res;
  }

  /** \brief returns all input ports
  */
  QVector<AlgPort*> AlgItem::getInputPorts()  const
  {
    QVector<AlgPort*> res;
    for (auto* p : mInputPorts)
    {
      res.append(p);
    }
    return res;
  }

  /** \brief handle item changes 
  */
  QVariant AlgItem::itemChange(GraphicsItemChange, const QVariant& value)
  {
    return value;
  }

}
}