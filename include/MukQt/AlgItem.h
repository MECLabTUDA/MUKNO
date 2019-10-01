#pragma once
#include "muk_qt_api.h"

#include <QtWidgets/QGraphicsPathItem>

namespace gris
{
  namespace muk
  {
    class AlgPort;

    /** \brief A visualization of a node in the algorithm pipeline

      Shown as a rounded rectangle within the graphics scene.
      The #AlgItem is also owner of the respective algorithm's ports as well as other details (e.g. buttons for interaction).
      The #AlgItem keeps information about the respective algorithm's ID, name and alias
    */
    class MUK_QT_API AlgItem : public QGraphicsPathItem
    {
      public:
        enum 
        {
          Type = QGraphicsItem::UserType + 1 
        };

      public:
        AlgItem(QGraphicsItem *parent = 0);

        QGraphicsPathItem* closestSubItem(const QPointF& pos);

      public:
        virtual int  type() const  Q_DECL_OVERRIDE { return Type; }
        virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) Q_DECL_OVERRIDE;

      public:
        void rebuild();
        void setName(const std::string& name)  { mAlgorithmName = name.c_str(); }
        void setAlias(const std::string& name) { mAlias = name.c_str(); }
        void addPort(bool isOutput, int ptr = 0);
        void addInputPort();
        void addOutputPort();
        void addInputPorts(size_t n);
        void addOutputPorts(size_t n);
        
        QVector<AlgPort*> getPorts() const;
        QVector<AlgPort*> getOutputPorts() const;
        QVector<AlgPort*> getInputPorts()  const;
        void              setRefID(int newId)        { mID = newId; }
        int               getRefID()           const { return mID; }

      public:
        AlgItem* clone() const;

      protected:
        virtual QVariant itemChange(GraphicsItemChange change, const QVariant& value) Q_DECL_OVERRIDE;
        
      private:        
        // algorithm data
        QString mAlgorithmName;
        QString mAlias;
        int     mID;             /// contains the corresponding filter id of an MukAlgorithm/AlgorithmWrapper        
        std::vector<AlgPort*> mInputPorts;
        std::vector<AlgPort*> mOutputPorts;
        QGraphicsTextItem* mLabelName;
        QGraphicsTextItem* mLabelAlias;

        // visualization
        QColor mBoxColor;
        QColor mBoxColorSelected;
        QColor mBoxFrameColor;
        float  mHorizontalMargin;
        float  mVerticalMargin;
        float  mVerticalNameMargin;
        float  mPortMargin;
        float  mWidth;
        float  mHeight;
    };
  }
}