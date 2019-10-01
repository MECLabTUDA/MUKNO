#include "private/muk.pch"
#include "TabImaging.h"

#include "AlgGraphicsView.h"
#include "AlgItem.h"
#include "AlgItemHandler.h"
#include "AlgPort.h"
#include "MukQRightClickMenu.h"

#include "MukCommon/MukException.h"

#include <QtWidgets>
#include <QMouseEvent>
#include <qgraphicslinearlayout.h>
#include <qgraphicslayout.h>

#include <boost/regex.hpp>

namespace gris
{
namespace muk
{
  /**
  */
  TabImaging::TabImaging(QWidget* parent)
    : QWidget(parent)
  {
    {
      QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
      sizePolicy.setHorizontalStretch(1);
      sizePolicy.setVerticalStretch(0);
      setSizePolicy(sizePolicy);
    }
    // major layout
    auto layout = new QVBoxLayout(this);

    // add QListWidget to Layout
    mpListWidget = new QListWidget(this);
    layout->addWidget(mpListWidget);
    mpFilterLine = new QLineEdit(this);
    layout->addWidget(mpFilterLine);
    layout->addItem(new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Preferred));

    connect(mpFilterLine, &QLineEdit::textChanged, this, &TabImaging::filterListWidget);

    mpView = new AlgGraphicsView(this);
    {
      mpScene = new QGraphicsScene(this);
      mpView->setScene(mpScene);
    }
    layout->addWidget(mpView);

    mpAlgItemHandler = new AlgItemHandler(this);
    mpAlgItemHandler->install(mpScene);

    // save/ load button

    auto gridLayout = new QGridLayout();
    layout->addLayout(gridLayout);
    {
      mpSaveAlgorithmButton = new QPushButton(this);
      mpSaveAlgorithmButton->setText("Save Algorithm");
      connect(mpSaveAlgorithmButton, &QPushButton::clicked, this, &TabImaging::saveAlgorithmSignal);
      gridLayout->addWidget(mpSaveAlgorithmButton, 0, 0);
    }
    {
      mpLoadAlgorithmButton = new QPushButton(this);
      mpLoadAlgorithmButton->setText("Load Algorithm");
      connect(mpLoadAlgorithmButton, &QPushButton::clicked, this, &TabImaging::loadAlgorithmSignal);
      gridLayout->addWidget(mpLoadAlgorithmButton, 0, 1);
    }
    {
      mpStartAlgorithmButton = new QPushButton(this);
      mpStartAlgorithmButton->setText("Start");
      connect(mpStartAlgorithmButton, &QPushButton::clicked, this, &TabImaging::startAlgorithmSignal);
      gridLayout->addWidget(mpStartAlgorithmButton, 1, 0);
    }
    {
      mpClearSceneButton = new QPushButton(this);
      mpClearSceneButton->setText("New");
      connect(mpClearSceneButton, &QPushButton::clicked, this, &TabImaging::clearSceneClicked);
      gridLayout->addWidget(mpClearSceneButton, 1, 1);
    }
    // setup connections
    setupConnections();
  }
  /**
  */
  TabImaging::~TabImaging()
  {
  }

  /** \brief setup all (qt) connections
  */
  void TabImaging::setupConnections()
  {
    connect(mpView,           &AlgGraphicsView::filterDropped,    this, &TabImaging::textDroppedInView);
    connect(mpView,           &AlgGraphicsView::textDropped,      this, &TabImaging::textDroppedInView);
    connect(mpAlgItemHandler, &AlgItemHandler::deleteBlock,       [&] (unsigned int id)          { emit this->algorithmDeleted(id); });
    connect(mpAlgItemHandler, &AlgItemHandler::connectionAdded,   [&] (AlgPort *p1, AlgPort *p2) { emit this->connectionAdded(p1, p2); });
    connect(mpAlgItemHandler, &AlgItemHandler::connectionDeleted, [&] (AlgPort *p1, AlgPort *p2) { emit this->connectionDeleted(p1, p2); });
    connect(mpAlgItemHandler, &AlgItemHandler::blockClicked,      [&] (unsigned int id)          { emit this->showAlgProperties(id); });
    connect(mpAlgItemHandler, &AlgItemHandler::portClicked,       [&] (unsigned int idAlgo, bool isOutput, unsigned int idPort) { emit this->showOutput(idAlgo, isOutput, idPort); });
  }

  /** \brief initialize TapImaging
  */
  void TabImaging::clear()
  {
    mpListWidget->clear();
    clearAlgorithms();
  }

  /** \brief initialize TapImaging
  */
  void TabImaging::clearAlgorithms()
  {
    mpScene->clear();
  }

  /** \brief set algorithm list in list widget
  */
  void TabImaging::setAlgorithmList(const std::vector<std::string>& list)
  {
    mpListWidget->clear();
    for (auto it = list.begin(); it != list.end(); ++it)
    {
      mpListWidget->addItem(QString::fromStdString(*it));
    }
    mpListWidget->sortItems();
    mpListWidget->setDragEnabled(true);
    mpListWidget->setMaximumHeight(150); // magic number? auto adjust creates inapproriate size, maybe 1/5 of tabImaging height?
  }

  /** \brief add new module (algorithm) to the scene

    \param[i] position position of the item in scene coordinates
  */
  void TabImaging::addAlgItem(unsigned int id, const std::string& name, const std::string& alias, std::size_t inPorts, std::size_t outPorts, const QPointF& position)
  {
    // Add new a new block for the filter and set the id of the filter
    AlgItem *b = new AlgItem(nullptr);
    b->setRefID(id);
    b->setX(position.x());
    b->setY(position.y());
    b->setName(name);
    b->setAlias(alias);
    mpScene->addItem(b);
    b->addInputPorts(inPorts);
    b->addOutputPorts(outPorts);
    b->rebuild();
    b->update();
  }

  /** \brief Adds an edge to the view based on the ids of src and target algorithm / port

    \param[in] srcAlgId     The ID of the source algorithm
    \param[in] srcPortId    The ID of the source algorithm's port
    \param[in] targetId     The ID of the target algorithm
    \param[in] targetPortId The ID of the target algorithm's port
  */
  void TabImaging::addAlgEdge(unsigned int srcAlgId, unsigned int srcPortId, unsigned int targetId, unsigned int targetPortId)
  {
    auto items = mpScene->items();
    auto iterSrc = std::find_if(items.begin(), items.end(), [&] (const auto* item)
      {
        auto* algItem = dynamic_cast<const AlgItem*>(item);
        if ( ! algItem)
          return false;
        return algItem->getRefID() == srcAlgId;
      });
    auto iterTarget = std::find_if(items.begin(), items.end(), [&] (const auto* item)
      {
        auto* algItem = dynamic_cast<const AlgItem*>(item);
        if ( ! algItem)
          return false;
        return algItem->getRefID() == targetId;
      });
    if (iterSrc == items.end() || iterTarget == items.end())
      return;
    // get AlgPorts
    auto* srcAlgItem = dynamic_cast<const AlgItem*>(*iterSrc);
    auto* tarAlgItem = dynamic_cast<const AlgItem*>(*iterTarget);
    auto* algSrcPort = srcAlgItem->getOutputPorts()[srcPortId];
    auto* algTarPort = tarAlgItem->getInputPorts()[targetPortId];
    auto* conn = new AlgConnection();
    conn->setOutputPort(algSrcPort);
    conn->setInputPort(algTarPort);
    conn->updatePosFromPorts();
    conn->updatePath();
    mpScene->addItem(conn);
  }

  /** \brief handle key press events
  */
  void TabImaging::keyPressEvent(QKeyEvent *event)
  {
    switch (event->key())
    {
      case Qt::Key_F5:
      {
        emit execute();
        break;
      }
    }
  }

  /**
  */
  void TabImaging::saveAlgorithmSignal()
  {
    QString fileName = QFileDialog::getSaveFileName(this,
      tr("Save Algorithm"), "",
      tr("Algorithm (*.alg);")); // ;;All Files (*)"
    if (fileName.isEmpty())
      return;
    
    emit saveAlgorithmClicked(fileName.toLocal8Bit().constData());
  }

  /**
  */
  void TabImaging::loadAlgorithmSignal()
  {
    QString fileName = QFileDialog::getOpenFileName(this,
      tr("Save Algorithm"), "",
      tr("Algorithm (*.alg);")); // ;;All Files (*)"
    if (fileName.isEmpty())
      return;

    emit loadAlgorithmClicked(fileName.toLocal8Bit().constData());
  }

  /**
  */
  void TabImaging::startAlgorithmSignal()
  {
	  emit execute();
  }

  /** \brief handles the drop event in AlgGraphicsView
  */
  void TabImaging::textDroppedInView(const QString& text, QPointF position) const
  {
    const auto str = text.toStdString();
    int id = mpAlgItemHandler->algorithmAt(position);
    if (id >= 0)
    {
      emit textDroppedOnAlgorithm(str, static_cast<unsigned int>(id));
    }
    else
    {
      emit textDroppedOnView(str, position);
    }
  }

  /**
  */
  void TabImaging::algorithmDroppedInView(const QString& text, QPointF position)
  {
    const auto algName = text.toStdString();
    emit textDroppedOnView(algName, position);
  }

  /** \brief Sets all items hidden that do not include the string #text
  */
  void TabImaging::filterListWidget(const QString& text)
  {
    if (text.isEmpty())
    {
      for (int i=0; i<mpListWidget->count(); ++i)
      {
        auto* item = mpListWidget->item(i);
        item->setHidden(false);
      }
      return;
    }
    auto str = text.toStdString();
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    for (int i = 0; i<mpListWidget->count(); ++i)
    {
      auto* item = mpListWidget->item(i);
      auto name = item->text().toStdString();
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      if (name.find(str) != std::string::npos)
        item->setHidden(false);
      else
        item->setHidden(true);
    }
  }
} // namespace muk
} // namespace gris
