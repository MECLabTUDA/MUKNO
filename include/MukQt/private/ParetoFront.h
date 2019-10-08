#pragma once

#include <QGraphicsScene.h>

#include <TabSelection.h>

namespace gris
{
  namespace muk
  {
    /** \brief Creates a ParetoFront with up to two Parameters which can be chosen by the user.
     The Front shows every filtered Path in the graphic and marks them with one of two colors.
     green if the path is efficient, meaning that no other path is better in both parameters at once.
     all other paths are colored lilac
    */
    class ParetoFront : public QGraphicsScene
    {
      Q_OBJECT
      signals:
        void clickedPath(size_t index);

      public:
        ParetoFront(size_t width, size_t height, QObject *parent = 0);

      public:
        size_t getClickedIndex() const { return mLastClickedIndex;}

        void setParameterList(bool is_param1, const QString& parameterName, const std::vector<double>* parameterList, const std::vector<size_t>* parameterOrder, const std::vector<size_t>& filteredPaths);

        void reload();

      protected:
        void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);
        void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *mouseEvent);

      private:
        double mWidth;
        double mHeight;
        QString mParam1Name;
        QString mParam2Name;
        size_t mLastClickedIndex;
        bool showsDetails = false;
        const std::vector<double>* mpParameter1List;
        const std::vector<double>* mpParameter2List;
        std::vector<std::tuple<double, double, size_t>> pathPositions;
        const std::vector<size_t>* mpParam1Order;
        const std::vector<size_t>* mpParam2Order;
        std::vector<size_t>  mFilteredPaths;
    };
  }
}
