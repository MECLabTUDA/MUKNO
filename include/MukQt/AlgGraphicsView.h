#pragma once

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QListWidget>

namespace gris
{
  namespace muk
  {
    /** \brief The QGraphicsView where the algorithm pipeline is visualized within.

      This class specifies certain interaction events like drag and drop or zoom.
    */
    class AlgGraphicsView : public QGraphicsView
    {
      Q_OBJECT

      public:
        AlgGraphicsView(QWidget* parent = nullptr);
        AlgGraphicsView(QGraphicsScene *scene, QWidget *parent = nullptr);

      signals:
        void filterDropped(const QString& text, const QPointF& position);
        void textDropped(const QString& str, QPointF position);

      public:
        //void 

      protected:
        void dropEvent(QDropEvent* event);
        void dragMoveEvent(QDragMoveEvent* event);
        void dragEnterEvent(QDragEnterEvent* event);
        void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;

      private:
       qreal h11 = 1.0;
       qreal h12 = 0;
       qreal h21 = 1.0;
       qreal h22 = 0;
    };
  }
}