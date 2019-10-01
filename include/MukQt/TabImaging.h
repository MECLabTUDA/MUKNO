#pragma once
#include "muk_qt_api.h"
#include "AlgConnection.h"
#include "AlgItemHandler.h"

#include <QtWidgets>

namespace gris
{
  namespace muk
  {
    class AlgGraphicsView;

    /**
    */
    class MUK_QT_API TabImaging : public QWidget
    {
      Q_OBJECT

      public:
        TabImaging(QWidget* parent = nullptr);
        ~TabImaging();

      signals:
        void execute();
        void algorithmDeleted(unsigned int id);
        void connectionAdded(AlgPort *p1, AlgPort *p2);
        void connectionDeleted(AlgPort *p1, AlgPort *p2);
        void showAlgProperties(unsigned int id);
        void showOutput(unsigned int idAlg, bool isOutput, unsigned int idPort);
        void saveAlgorithmClicked(const std::string& filename);
        void loadAlgorithmClicked(const std::string& filename);
        void clearSceneClicked();
        void textDroppedOnAlgorithm(const std::string& text, unsigned int) const;
        void textDroppedOnView(const std::string& text, QPointF position) const;

      public:
        void clear();
        void clearAlgorithms();
        void setupConnections();
        // QWidgetList
        void setAlgorithmList(const std::vector<std::string>& list);
        // QGraphicsScene
        void addAlgItem(unsigned int id, const std::string& name, const std::string& alias, std::size_t inPorts, std::size_t outPorts, const QPointF& position);
        void addAlgEdge(unsigned int srcAlgId, unsigned int srcPortId, unsigned int targetId, unsigned int targetPortId);

        const QGraphicsScene* getGraphicsScene() const { return mpScene; }
        const AlgItemHandler* getAlgItemHandler() const { return mpAlgItemHandler; }
        
      protected:
        void keyPressEvent(QKeyEvent *event);

      private:
        void saveAlgorithmSignal();
        void loadAlgorithmSignal();
        void startAlgorithmSignal();
        void algorithmDroppedInView(const QString& text, QPointF position);
        void textDroppedInView(const QString& text, QPointF position) const;
        void filterListWidget(const QString& text);

      private:
        QListWidget*      mpListWidget; // algorithmList
        QLineEdit*        mpFilterLine;
        QGraphicsScene*   mpScene;
        AlgGraphicsView*  mpView;
        AlgItemHandler*   mpAlgItemHandler;
        QPushButton*      mpSaveAlgorithmButton;
        QPushButton*      mpLoadAlgorithmButton;
        QPushButton*      mpStartAlgorithmButton;
        QPushButton*      mpClearSceneButton;
    };
  }
}

