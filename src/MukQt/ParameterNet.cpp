#include "private/muk.pch"
#include "private/ParameterNet.h"

#include "MukCommon/gris_math.h"
#include "MukCommon/MukVector.h"

#include <QGraphicsSceneMouseEvent>
#include <QMouseEvent>
#include <QtWidgets/QSpinBox>
#include <qgraphicsitem.h>

namespace
{
  std::vector<double> pointOnCircle(size_t pi, size_t N, double rad, double center, double circleOffset);
}

namespace gris
{
namespace muk
{
  /** \brief sets the center and the radius of the ParameterNet, as well as the width and height
  */
  ParameterNet::ParameterNet(size_t width, size_t height, TabSelection *myTS, QObject *parent) : QGraphicsScene(parent)
  {
    this->setSceneRect(0, 0, width, height);
    middle = static_cast<int>(height / 2);
    radius = middle - 25;
    myTabSelection = myTS;
  }

  /** \brief Reloads the Values of the ParamterNet, sets the valueBars, the label and the spannedArea
  */
  void ParameterNet::reloadValues()
  {
    // checks if its a new ParameterNet
    if (wasClear)
      wasClear = false;
    else
    {
      // searches for all the existing items created by reloadValues and removes them
      for (size_t i(0); i < N * 2 + 1; ++i)
      {
        this->removeItem(this->items(Qt::AscendingOrder).last());
      }
    }
    // Endpoint of the Path for the spannedArea
    QPointF pathEnd = { 0,0 };
    auto* path = new QPainterPath(pathEnd);
    auto* spannedArea = new QGraphicsPathItem();
    spannedArea->setBrush(QColor(161, 217, 244)); // light medizin blue
    spannedArea->setOpacity(0.3);
    this->addItem(spannedArea);
    size_t index(0);
    for (size_t i(0); i < N; ++i)
    {
      // Ignore all inactive obstacles
      while (!myActiveObstacles[index])
        ++index;
      auto* valueBars = new QGraphicsRectItem(-10, -5, 20, 10);
      valueBars->setTransformOriginPoint(0, 0);
      // rotation for every valueBar according to their scale
      auto rotation = -45 - static_cast<int>(360 / N * i);
      valueBars->setRotation(rotation);
      auto Pos = pointOnCircle(i, N, radius * myValues[index], middle, 0.25);
      valueBars->setPos(Pos[0], Pos[1]);
      valueBars->setPen(QPen(Qt::darkGray));
      valueBars->setBrush(myWeightingColors[index]);
      auto* label = new QGraphicsTextItem(QString("%1: %2").arg(myNames[index]).arg(myValues[index]));
      // scales the label with bigger N for a better overview
      label->setTransform(QTransform::fromScale(1 - 0.03*N, 1 - 0.03*N), true);
      Pos = pointOnCircle(i, N, radius, middle, 0.25);
      double widthShift = label->sceneBoundingRect().width() / 2;
      double heightShift = label->sceneBoundingRect().height() / 2;
      // shifts the label to the left or the right if it would go out of bounds, small adjustments because the Nets have more width then height in the Selection-Tab
      double distToRight = std::min(0.0, middle * 2 - Pos[0] - widthShift);
      double distToLeft = std::min(0.0, Pos[0] - widthShift);
      // put the label over the scale if it's in the upper half of the Net and under the scale if it's in the lower half
      const double heightRatio = 20;
      const double heightRatioConstant = 35;
      if (Pos[1] < middle)
        label->setPos(Pos[0] - widthShift + distToRight - distToLeft, Pos[1] - heightShift - heightRatioConstant + std::fabs(Pos[1] - middle) / middle * heightRatio);
      else
        label->setPos(Pos[0] - widthShift + distToRight - distToLeft, Pos[1] - heightShift + heightRatioConstant - (Pos[1] - middle) / middle * heightRatio);
      this->addItem(label);
      // sets the starting point for the path of the spanned Area
      if (i == 0)
      {
        pathEnd = valueBars->pos();
        path->moveTo(pathEnd);
      }
      else
      {
        path->lineTo(valueBars->pos());
      }
      if (i == N - 1)
        path->lineTo(pathEnd);
      this->addItem(valueBars);
      ++index;
    }
    spannedArea->setPath(*path);
  }

  /** \brief grabbs a valueBar when clicked with the left mousebutton
  */
  void ParameterNet::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
  {
    if (mouseEvent->button() != Qt::LeftButton)
      return;
    auto itemList = this->items(Qt::DescendingOrder);
    // determines the clicked bar, the order in which the items are created is known so they can be found like this
    for (int i(1); i < N * 2 + 1; i += 2)
    {
      if (itemList[i]->sceneBoundingRect().contains(mouseEvent->scenePos()))
      {
        grabbedBar = N - ((i) / 2) - 1;
        grabbedSomething = true;
      }
    }
  }

  /** when double clicking on the parameterNet the closest scale will be determined and setBarToPoint will be called with it
  */
  void ParameterNet::mouseDoubleClickEvent(QGraphicsSceneMouseEvent * mouseEvent)
  {
    if (mouseEvent->button() != Qt::LeftButton)
      return;
    auto itemList = this->items(Qt::SortOrder::AscendingOrder);
    // check which of the scale Endpoints is closest to the mouse => get the correct Bar
    double dist = std::numeric_limits<double>::infinity();
    size_t nearestScaleInd;
    for (size_t i(0); i < N; i++)
    {
      auto Pos = pointOnCircle(i, N, radius, middle, 0.25);
      Vec2d point = { Pos[0], Pos[1] };
      Vec2d mousePos = { mouseEvent->scenePos().x(), mouseEvent->scenePos().y() };
      auto tempDistance = (mousePos - point).norm();
      if (tempDistance < dist)
      {
        dist = tempDistance;
        nearestScaleInd = i;
      }
    }
    setBarToPoint(mouseEvent->scenePos(), nearestScaleInd);
  }

  /**

    When a bar was clicked and the mouse moves, the closest point on the corresponding scale gets identified, 
    the value is adjusted accordingly and the values of the ParameterNet are reloaded
  */
  void ParameterNet::mouseMoveEvent(QGraphicsSceneMouseEvent * mouseEvent)
  {
    if (grabbedSomething)
    {
      setBarToPoint(mouseEvent->scenePos(), grabbedBar);
    }
  }

  /**
      ends the grab | drag and drop
  */
  void ParameterNet::mouseReleaseEvent(QGraphicsSceneMouseEvent * mouseEvent)
  {
    grabbedSomething = false;
  }

  /** sets the Bar with grabbedBarInd as its Index to the position closed to mousePosition on the corrosponding scale
  */
  void ParameterNet::setBarToPoint(QPointF mousePosition, size_t grabbedBarInd)
  {
    double distance = std::numeric_limits<double>::infinity();
    double value(0.0);
    // checks every point on the corresponding scale for the lowest distance to the position of the mouse
    for (double i(0); i < 1.01; i += 0.01)
    {
      double tempDistance;
      auto Pos = pointOnCircle(grabbedBarInd, N, radius * i, middle, 0.25);
      Vec2d point = { Pos[0], Pos[1] };
      Vec2d mousePos = { mousePosition.x(), mousePosition.y() };
      tempDistance = (mousePos - point).norm();
      if (tempDistance < distance)
      {
        distance = tempDistance;
        value = i;
      }
    }
    this->setValue(grabbedBarInd, value);
    this->reloadValues();
    // applies the changes by updating the corresponding spinboxes signalling the weightings in SelectionLogic
    if (isObstacleNet)
      myTabSelection->findChild<QDoubleSpinBox*>(QString("OValue %1").arg(grabbedBarInd))->setValue(value);
    else
      myTabSelection->findChild<QDoubleSpinBox*>(QString("CValue %1").arg(grabbedBarInd))->setValue(value);
  }

  /** \brief Reloads the whole ParameterNet
  
    starts with the center, the scales, the scaleBars and the annotations. Then calls reloadValues for the rest
  */
  void ParameterNet::reload()
  {
    this->clear();
    wasClear = true;
    auto* center = new QGraphicsEllipseItem(-5, -5, 10, 10);
    center->setPos(middle, middle);
    center->setBrush(Qt::darkGray);
    center->setPen(QPen(Qt::black));
    this->addItem(center);
    if (N == 0)
    {
      return;
    }
    // Sets N Scales around the center
    for (size_t i(0); i < N; ++i)
    {
      auto Pos = pointOnCircle(i, N, radius, middle, 0.25);
      auto* scale = new QGraphicsLineItem(middle, middle, Pos[0], Pos[1]);
      scale->setPen(QPen(Qt::black, 2));
      this->addItem(scale);
      // Sets the 4 annotations on every Scale
      for (size_t j(1); j < 5; ++j)
      {
        auto* annotation = new QGraphicsTextItem(QString("%1").arg(j*0.25));
        // scales the annotations with the size of N for better overview
        annotation->setTransform(QTransform::fromScale(1 - 0.035*N, 1 - 0.035*N), true);
        // circleOffset needs to be adjusted for the annotations because of the differing radii
        Pos = pointOnCircle(i, N, radius * 0.25*j, middle, 0.275 + 0.025*std::pow(5 - j, 1.1));
        annotation->setPos(Pos[0] - annotation->sceneBoundingRect().width() / 2, Pos[1] - annotation->sceneBoundingRect().height() / 2);
        // 2 points for the scaleBar using different circleOffsets. Automaticly makes the scaleBars with higher Values bigger
        Pos = pointOnCircle(i, N, radius * 0.25*j, middle, 0.22);
        auto Pos2 = pointOnCircle(i, N, radius*0.25*j, middle, 0.28);
        auto* scaleBars = new QGraphicsLineItem(Pos[0], Pos[1], Pos2[0], Pos2[1]);
        if (j == 4)
          scaleBars->setPen(QPen(Qt::black, 2));
        // hide some annotations when N is big for better overview
        this->addItem(annotation);
        if (N >= 11)
          annotation->hide();
        else if (N >= 6 && (j == 1 || j == 3))
          annotation->hide();
        this->addItem(scaleBars);
      }
    }
    reloadValues();
  }

  void ParameterNet::setColor(const size_t index, const size_t red, const size_t green, const size_t blue) 
  {
    auto r = static_cast<int>(red);
    auto g = static_cast<int>(green);
    auto b = static_cast<int>(blue);
    setColor(index, QColor(r,g,b));
  }
}
}

namespace
{
  /** \brief Auxiliary function to determine the i-th point of N equi-distant points on a circle with radius rad
  and center at (center|center). CircleOffset shifts the point on the circle.

  \param i  the i-th point
  \param N  Number of equidistant points 
  \param rad radius of circle
  \param center center of circle (center|center)
  \param circleOffset shifts the point on the circle (somehow)
  */
  std::vector<double> pointOnCircle(size_t i, size_t N, double rad, double center, double circleOffset)
  {
    using namespace gris;
    std::vector<double> result(2);
    result[0] = ::sin((i * 2 * M_Pi) / N + circleOffset * M_Pi) * rad + center;
    result[1] = ::cos((i * 2 * M_Pi) / N + circleOffset * M_Pi) * rad + center;
    return result;
  }
}