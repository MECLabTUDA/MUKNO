#include "private/muk.pch"
#include "private/ParetoFront.h"

#include "MukCommon/gris_math.h"
#include "MukCommon/MukVector.h"
#include "MukCommon/MukException.h"

#include <QGraphicsSceneMouseEvent>
#include <QMouseEvent>
#include <QtWidgets/QSpinBox>
#include <qgraphicsitem.h>
#include <qtransform.h>

#include <boost/format.hpp>

#include <tuple>

namespace gris
{
  namespace muk
  {
    /** \brief sets width and height of the Widget and also initializes all other memberVariables.
      Also loads everything thats always the same for all parameters (background, axis and stuff)
    */
    ParetoFront::ParetoFront(size_t width, size_t height, QObject *parent) : QGraphicsScene(parent), mWidth(width), mHeight(height)
    {
      this->setSceneRect(0, 0, mWidth, mHeight);
      mParam1Name = "- not selected -";
      mParam2Name = "- not selected -";
      mLastClickedIndex = -1;
      mpParameter1List = nullptr;
      mpParameter2List = nullptr;
      mpParam1Order = nullptr;
      mpParam2Order = nullptr;
      auto* background = new QGraphicsRectItem(0.15 * mWidth, 0.05 * mHeight, 0.8 * mWidth, 0.8 * mHeight);
      background->setBrush(QColor(230, 230, 230));
      background->setPen(QPen(QColor(255, 255, 255, 0), 0));
      this->addItem(background);
      auto* origin = new QGraphicsEllipseItem(0.15 * mWidth - 5, 0.85 * mHeight - 5, 10, 10);
      origin->setBrush(QColor(0, 0, 0));
      this->addItem(origin);
      auto* xAxis = new QGraphicsLineItem(0.15 * mWidth, 0.85 * mHeight, 0.95 * mWidth, 0.85 * mHeight);
      xAxis->setPen(QPen(QColor(0, 0, 0), 3));
      this->addItem(xAxis);
      auto* yAxis = new QGraphicsLineItem(0.15 * mWidth, 0.85 * mHeight, 0.15 * mWidth, 0.05 * mHeight);
      yAxis->setPen(QPen(QColor(0, 0, 0), 3));
      this->addItem(yAxis);
      auto* ArrowLine = new QGraphicsLineItem(0.95 * mWidth, 0.85 * mHeight, 0.925 * mWidth, 0.825 * mHeight);
      ArrowLine->setPen(QPen(QColor(0, 0, 0), 3));
      this->addItem(ArrowLine);
      ArrowLine = new QGraphicsLineItem(0.95 * mWidth, 0.85 * mHeight, 0.925 * mWidth, 0.875 * mHeight);
      ArrowLine->setPen(QPen(QColor(0, 0, 0), 3));
      this->addItem(ArrowLine);
      ArrowLine = new QGraphicsLineItem(0.15 * mWidth, 0.05 * mHeight, 0.125 * mWidth, 0.075 * mHeight);
      ArrowLine->setPen(QPen(QColor(0, 0, 0), 3));
      this->addItem(ArrowLine);
      ArrowLine = new QGraphicsLineItem(0.15 * mWidth, 0.05 * mHeight, 0.175 * mWidth, 0.075 * mHeight);
      ArrowLine->setPen(QPen(QColor(0, 0, 0), 3));
      this->addItem(ArrowLine);
      auto* valueLine = new QGraphicsLineItem(0.25 * mWidth, 0.84 * mHeight, 0.25 * mWidth, 0.87 * mHeight);
      valueLine->setPen(QPen(QColor(0, 0, 0), 2));
      this->addItem(valueLine);
      valueLine = new QGraphicsLineItem(0.85 * mWidth, 0.84 * mHeight, 0.85 * mWidth, 0.87 * mHeight);
      valueLine->setPen(QPen(QColor(0, 0, 0), 2));
      this->addItem(valueLine);
      valueLine = new QGraphicsLineItem(0.16 * mWidth, 0.15 * mHeight, 0.13 * mWidth, 0.15 * mHeight);
      valueLine->setPen(QPen(QColor(0, 0, 0), 2));
      this->addItem(valueLine);
      valueLine = new QGraphicsLineItem(0.16 * mWidth, 0.75 * mHeight, 0.13 * mWidth, 0.75 * mHeight);
      valueLine->setPen(QPen(QColor(0, 0, 0), 2));
      this->addItem(valueLine);
      reload();
    }

    /** called by SelectionController when one of the parameters is chosen in the ParetoWindow.
      sets the Name, the Values and the Ordering of one of the two parameters
      reloads the ParetoFront afterwards
    */
    void ParetoFront::setParameterList(bool is_param1, const QString & parameterName, const std::vector<double>* parameterList, const std::vector<size_t>* parameterOrder, const std::vector<size_t>& filteredPaths)
    {
      if (is_param1)
      {
        mParam1Name = parameterName;
        mpParameter1List = parameterList;
        mpParam1Order = parameterOrder;
      }
      else
      {
        mParam2Name = parameterName;
        mpParameter2List = parameterList;
        mpParam2Order = parameterOrder;
      }
      mFilteredPaths = filteredPaths;
      reload();
    }

    /** Clears everything drawn in the scene and redraws the Axis (could be cut when clearing all but the base and not the whole thing)
      If both ParameterList are not nullptr all filtered Paths are drawn in the scene
      The most effecient Paths regarding the parameters are drawn green and make up the ParetoFront
      All others are drawn lilac
    */
    void ParetoFront::reload()
    {
      auto itemList = this->items(Qt::AscendingOrder);
      // remove all items except the 12 base items set by the constructor
      for (size_t i(12); i < itemList.size(); ++i)
      {
        this->removeItem(itemList[int(i)]);
      }
      showsDetails = false;
      pathPositions.clear();

      auto* axisName = new QGraphicsTextItem(mParam1Name);
      auto rotator = QTransform();
      rotator.rotate(90);
      {
        axisName->setScale(1.3);
        axisName->setPos(0.05 * mWidth, 0.45 * mHeight - axisName->sceneBoundingRect().width() / 2);
        axisName->setTransform(rotator);
        this->addItem(axisName);
        axisName = new QGraphicsTextItem(mParam2Name);
        axisName->setScale(1.3);
        axisName->setPos(0.55 * mWidth - axisName->sceneBoundingRect().width() / 2, 0.95 * mHeight);
        this->addItem(axisName);
      }
      // when both parameters are set
      if (mpParameter1List && mpParameter2List)
      {
        double parameter1Worst(0);
        double parameter2Worst(0);
        double parameter1Best(0);
        double parameter2Best(0);
        {
          // ugly way to check for the worst and best of the parameters while ignoring all filtered out paths
          for (size_t i : *mpParam1Order)
          {
            if (std::find(mFilteredPaths.begin(), mFilteredPaths.end(), i) == mFilteredPaths.end())
              continue;
            parameter1Best = mpParameter1List->at(i);
            break;
          }
          for (size_t i : *mpParam2Order)
          {
            if (std::find(mFilteredPaths.begin(), mFilteredPaths.end(), i) == mFilteredPaths.end())
              continue;
            parameter2Best = mpParameter2List->at(i);
            break;
          }
          for (int i(int(mpParam1Order->size()) - 1); i > -1; --i)
          {
            if (std::find(mFilteredPaths.begin(), mFilteredPaths.end(), mpParam1Order->at(i)) == mFilteredPaths.end())
              continue;
            parameter1Worst = mpParameter1List->at(mpParam1Order->at(i));
            break;
          }
          for (int i(int(mpParam2Order->size()) - 1); i > -1; --i)
          {
            if (std::find(mFilteredPaths.begin(), mFilteredPaths.end(), mpParam2Order->at(i)) == mFilteredPaths.end())
              continue;
            parameter2Worst = mpParameter2List->at(mpParam2Order->at(i));
            break;
          }
        }
        auto parameter1Diff = parameter1Best - parameter1Worst;
        auto parameter2Diff = parameter2Best - parameter2Worst;

        double bestXPos = 0.0;
        //char buffer[50];
        std::string buffer;
        buffer = (boost::format("Best: %2.2f") % parameter1Best).str();
        auto* axisMark = new QGraphicsTextItem(buffer.c_str());
        {
          axisMark->setScale(1.3);
          axisMark->setPos(0.1 * mWidth, 0.15 * mHeight - axisMark->sceneBoundingRect().width() / 2);
          axisMark->setTransform(rotator);
          this->addItem(axisMark);
        }
        buffer = (boost::format("Worst: %2.2f") % parameter1Worst).str();
        axisMark = new QGraphicsTextItem(buffer.c_str());
        {
          axisMark->setScale(1.3);
          axisMark->setPos(0.1 * mWidth, 0.75 * mHeight - axisMark->sceneBoundingRect().width() / 2);
          axisMark->setTransform(rotator);
          this->addItem(axisMark);
        }
        buffer = (boost::format("Best: %2.2f") % parameter2Best).str();
        axisMark = new QGraphicsTextItem(buffer.c_str());
        {
          axisMark->setScale(1.3);
          axisMark->setPos(0.85 * mWidth - axisMark->sceneBoundingRect().width() / 2, 0.9 * mHeight);
          this->addItem(axisMark);
        }
        buffer = (boost::format("Worst: %2.2f") % parameter2Worst).str();
        axisMark = new QGraphicsTextItem(buffer.c_str());
        {
          axisMark->setScale(1.3);
          axisMark->setPos(0.25 * mWidth - axisMark->sceneBoundingRect().width() / 2, 0.9 * mHeight);
          this->addItem(axisMark);
        }
        // Line for the efficient values
        auto* multiLine = new QGraphicsPathItem();
        multiLine->setPen(QPen(QColor(0, 200, 50), 3));
        this->addItem(multiLine);
        auto path = std::make_unique<QPainterPath>(QPoint(0, 0));

        bool first = true;
        for (size_t i(0); i < mpParam1Order->size(); ++i)
        {
          if (std::find(mFilteredPaths.begin(), mFilteredPaths.end(), mpParam1Order->at(i)) == mFilteredPaths.end())
            continue;
          // on the xAxis the Values from 0.25 to 0.85 are given to the parameter2
          // and on the yAxis the Values from 0.75 to 0.15 are given to the parameter1
          auto xPos = (0.25 + 0.6 * ((mpParameter2List->at(mpParam1Order->at(i)) - parameter2Worst) / parameter2Diff));
          auto yPos = (0.75 - 0.6 * ((mpParameter1List->at(mpParam1Order->at(i)) - parameter1Worst) / parameter1Diff));
          pathPositions.push_back(std::make_tuple(xPos * mWidth, yPos * mHeight, mpParam1Order->at(i)));
          auto* mark = new QGraphicsEllipseItem(xPos * mWidth - 5, yPos * mHeight - 5, 11, 11);
          // since the for-loop iterates over the parameter1List from best to worst the next value is automaticly worse then the one before in that parameter
          // so in order to be efficient too it has to be better in the other parameter then all that came before him
          if (xPos > bestXPos)
          {
            // green color
            mark->setBrush(QColor(0, 200, 50));
            bestXPos = xPos;
            if (first)
            {
              path->moveTo(xPos * mWidth, yPos * mHeight);
              first = false;
            }
            else
              path->lineTo(xPos * mWidth, yPos * mHeight);
          }
          else
          {
            // lilac color
            mark->setBrush(QColor(200, 100, 255));
          }
          this->addItem(mark);
        }
        multiLine->setPath(*path);
      }
    }

    /** When the mouse is pressed on an pathPoint detailedText appears under it.
    */
    void ParetoFront::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
    {
      if (mouseEvent->button() != Qt::LeftButton)
        return;
      if (showsDetails)
      {
        auto itemList = this->items(Qt::DescendingOrder);
        this->removeItem(itemList[0]);
        showsDetails = false;
      }
      auto mousePos = mouseEvent->scenePos();
      for (std::tuple<double, double, size_t> pos : pathPositions)
      {
        if (sqrt(pow(mousePos.x() - std::get<0>(pos), 2) + pow(mousePos.y() - std::get<1>(pos), 2)) <= 5)
        {
          mLastClickedIndex = std::get<2>(pos);
          char buffer[200];
          sprintf(buffer, "Path Index: %d\n%s: %f\n%s: %f", int(mLastClickedIndex), mParam1Name.toStdString().c_str(), mpParameter1List->at(mLastClickedIndex), mParam2Name.toStdString().c_str(), mpParameter2List->at(mLastClickedIndex));
          auto* detailText = new QGraphicsTextItem(buffer);
          detailText->setPos(std::get<0>(pos) - detailText->sceneBoundingRect().width() / 2, std::get<1>(pos) + 10);
          this->addItem(detailText);
          showsDetails = true;
          break;
        }
      }
      if (!showsDetails)
        mLastClickedIndex = -1;
    }

    /** When a Path is double-clicked it is selected in the MainWindow
    */
    void ParetoFront::mouseDoubleClickEvent(QGraphicsSceneMouseEvent * mouseEvent)
    {
      if (mouseEvent->button() != Qt::LeftButton)
        return;
      if(mLastClickedIndex != -1)
        emit this->clickedPath(mLastClickedIndex);
    }
  }
}