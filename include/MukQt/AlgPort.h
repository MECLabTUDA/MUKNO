#pragma once
#include "muk_qt_api.h"

#include <QtWidgets/QGraphicsPathItem>

namespace gris
{
  namespace muk
  {
    class AlgConnection;
    class AlgItem;

    /** \brief A visualization of an algorithm's input or output port

      At the moment, shown as a green circle at the top or bottom of an #AlgItem.
      The #AlgItem is responsible of the positioning.
      This class keeps information about its AlgItem, its connection and its type (output/input).
    */
    class MUK_QT_API AlgPort : public QGraphicsPathItem
    {
      public:
        enum
        {
          Type = QGraphicsItem::UserType + 2
        };

      public:
        AlgPort(bool isOutput, QGraphicsItem* parent = nullptr);
        ~AlgPort();
        
      protected:
        virtual int  type() const  Q_DECL_OVERRIDE { return Type; }
        virtual QVariant itemChange(GraphicsItemChange change, const QVariant& value);

      public:
        int     getRefID()                  const;
        void    setPortID(int id)                 { mPortId = id; };
        int     getPortID()                 const { return mPortId; };
        quint64 getPtr()                    const { return mPtr; }
        void    setPtr(quint64 p)                 { mPtr = p; }
        bool    isOutput()                  const { return mIsOutput; }

      public:
        void                            setAlgItem(AlgItem* p)          { mpAlgItem = p; }
        AlgItem*                        getAlgItem()             const  { return mpAlgItem; }
        QVector<AlgConnection*>&        getConnections()                { return mConnections; }
        const QVector<AlgConnection*>&  getConnections()          const { return mConnections; }

      public:
        void setPortColor(const QColor& color)              { mPortColor = color; }
        void setPortFrameColor(const QColor& color)        { mPortFrameColor = color; }
        int  getPortRadius()                          const { return mPortRadius; }
        bool isConnected(AlgPort*)                    const;

      private:
        // graph data
        int       mPortId;  /// the port id
        bool      mIsOutput;
        AlgItem*  mpAlgItem;
        QVector<AlgConnection*> mConnections;

        // visualization
        int       mPortRadius;
        QColor    mPortColor;
        QColor    mPortFrameColor;
        quint64   mPtr;
    };
  }
}