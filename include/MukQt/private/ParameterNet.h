#pragma once

#include <QGraphicsScene.h>

#include <TabSelection.h>
//#include <qgraphicsview.h>

namespace gris
{
  namespace muk
  {

    /** \brief Displays a ParameterNet

      this is, e.g., a star plot
    */
    class ParameterNet : public QGraphicsScene
    {
      //Q_OBJECT

      public:
        ParameterNet(size_t width, size_t height, TabSelection *myTS, QObject *parent = 0);

      public:
        std::vector<QColor> getColors()             const { return myWeightingColors; }
        QColor              getColor(size_t index)  const { return myWeightingColors[index]; }
        std::vector<double> getValues()             const { return myValues; }
        double              getValue(size_t index)  const { return myValues[index]; }
        std::vector<QString> getNames()             const { return myNames; }
        QString              getName(size_t index)  const { return myNames[index]; }
        bool                getActiveObstacle(size_t index) const { return myActiveObstacles[index]; }
        std::vector<bool>   getActiveObstacles() const { return myActiveObstacles; }
        size_t              getN()                  const { return N; }

        void  setColor(const size_t index, const size_t red, const size_t green, const size_t blue);
        void  setColor(const size_t index, const QColor color) { myWeightingColors[index] = color; }
        void  setColors(const std::vector<QColor> colors) { myWeightingColors = colors; }
        void  setValue(const size_t index, const double value) { myValues[index] = value; }
        void  setValues(const std::vector<double> values) { myValues = values; }
        void  setName(const size_t index, const QString name) { myNames[index] = name; }
        void  setNames(const std::vector<QString> names) { myNames = names; }
        void  setActiveObstacle(const size_t index, const bool isActive) { myActiveObstacles[index] = isActive; }
        void  setActiveObstacles(const std::vector<bool> activeObstacles) { myActiveObstacles = activeObstacles; }
        void  setN(const size_t numberOfWeightings) { N = numberOfWeightings; }

      public:
        void setIsObstacleNet(const bool isON) { isObstacleNet = isON; }
        // reloads the complete ParameterNet
        void reload();
        // reloads the Values of the ParameterNet
        void reloadValues();

      protected:
        void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);
        void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *mouseEvent);
        void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent);
        void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent);

      private:        
        void setBarToPoint(QPointF mousePosition, size_t grabbedBarInd);

        std::vector<QColor> myWeightingColors;  /// Colors of the bar
        std::vector<double> myValues;           /// Values of the weights
        std::vector<QString> myNames;           /// Names of the scales
        std::vector<bool> myActiveObstacles;    /// Saves the activ obstacles that will be shown in the ParameterNet        
        size_t N;                               /// Number of the scales
        double middle;                             // Center of the ParameterNet (middle|middle)
        double radius;                             // Radius of the ParameterNet
        bool wasClear = true;                   // Is true if the ParameterNet was recently cleared  
        size_t grabbedBar = 0;                  // Index of the Bar grabbed by drag and drop
        bool isObstacleNet;
        bool grabbedSomething = false;
        TabSelection* myTabSelection;
    };
  }
}
