#include "private/muk.pch"
#include "QuickPathAnalysisWindow.h"

#include <qcustomPlot.h>

#include <qlayout.h>
#include <QSpacerItem>
#include <QTableWidget>

#include <boost/filesystem.hpp>

namespace
{
  void clearWidgets(QLayout * layout) 
  {
    if (! layout)
      return;
    while (auto item = layout->takeAt(0)) 
    {
      delete item->widget();
      clearWidgets(item->layout());
    }
  }
}

namespace
{
  using namespace gris::muk;

  /**
  */
  class MyPlot : public QCustomPlot
  {
    public:
      MyPlot(QWidget* parent = nullptr)
        : QCustomPlot(parent) 
      {}

    public:
      void mouseDoubleClickEvent(QMouseEvent *mouseEvent)
      {
        emit dynamic_cast<QuickPathAnalysisWindow*>(parent())->plotDoubleClicked(mouseEvent);
        mouseEvent->accept();
      }
  };
}

namespace gris
{
namespace muk
{
  /** \brief all kinds of cache
  */
  struct QuickPathAnalysisWindow::GraphItemContainer
  {
    public:
      struct GraphData
      {
        QCPGraph*     graph;
        std::vector<QCPItemLine*>  lines;
        std::vector<QCPItemText*>  labels;
      };

    public:
      GraphData& addGraph()
      {
        graphs.push_back(GraphData());
        return graphs.back();
      }

    public:
      std::vector<GraphData> graphs;
      QCPItemLine*  safetyDistGraph;
      QCPItemText*  safetyDistText;
      QSpinBox*     idxSpinBox;
      QTableWidget* distanceTable;
      int           fontSize = 12;
  };

  // ============================================================================

  /**
  */
  QuickPathAnalysisWindow::QuickPathAnalysisWindow(QWidget* parent)
    : QDialog(parent)
    , mpContainer(std::make_unique<GraphItemContainer>())
    , mpLocalInfoLayout(nullptr)
  {
    // set non blocking
    setWindowModality(Qt::NonModal);
    Qt::WindowFlags flags = 0;
    flags |= Qt::WindowMaximizeButtonHint | Qt::WindowMinimizeButtonHint | Qt::WindowCloseButtonHint;
    setWindowFlags(flags);

    mpLayout = new QGridLayout(this);
    // on the left comes the plotw widget
    {
      mpPlot = new MyPlot(this);
      mpPlot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      mpPlot->yAxis->setRangeLower(0);
      mpPlot->yAxis->setLabel("distance to obstacles (mm)");
      //mpPlot->xAxis2->setLabel("distances");
      mpPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

      // safety distance
      auto safetyLine = new QCPItemLine(mpPlot);
      safetyLine->setPen(QPen(Qt::red));
      safetyLine->setClipAxisRect(mpPlot->axisRect());
      safetyLine->setClipToAxisRect(true);
      safetyLine->position("start")->setAxes(mpPlot->xAxis, mpPlot->yAxis);
      safetyLine->position("end")  ->setAxes(mpPlot->xAxis, mpPlot->yAxis);
      safetyLine->setHead(QCPLineEnding(QCPLineEnding::esNone));
      mpContainer->safetyDistGraph = safetyLine;

      auto textSafety = new QCPItemText(mpPlot);
      textSafety->setClipAxisRect(mpPlot->axisRect());
      textSafety->setClipToAxisRect(true);
      textSafety->position->setAxes(mpPlot->xAxis, mpPlot->yAxis);
      textSafety->setPositionAlignment(Qt::AlignVCenter | Qt::AlignRight);
      textSafety->setText("Safety\nDistance");
      textSafety->setPadding(QMargins(0, 0, 10, 0)); // thats the distance between "text and textbox"
      QFont font;
      textSafety->setFont(QFont(font.family(), mpContainer->fontSize));
      mpContainer->safetyDistText = textSafety;
    }
    mpLayout->addWidget(mpPlot, 0, 0);
    // on the right comes info and interaction
    auto* infoLayout = new QGridLayout();
    {
      int row(0);
      auto* table = new QTableWidget(1, 2, this);
      table->verticalHeader()->hide();
      table->horizontalHeader()->hide();
      table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
      table->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
      table->setSelectionMode(QTableWidget::MultiSelection);
      infoLayout->addWidget(table, row, 0, 1, 3);
      mpContainer->distanceTable = table;
      
      ++row;
      auto* nextLabel = new QLabel(this);
      nextLabel->setText("Add path: ");
      nextLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
      infoLayout->addWidget(nextLabel, row, 0);
      
      auto* spinBox = new QSpinBox(this);
      spinBox->setMinimum(0);
      spinBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
      infoLayout->addWidget(spinBox, row, 1);
      mpContainer->idxSpinBox = spinBox;

      ++row;
      auto nextButton = new QPushButton(this);
      nextButton->setText("Add Path");
      connect(nextButton, &QPushButton::clicked, [=] () { emit this->pathRequestedClicked(spinBox->value()); });
      infoLayout->addWidget(nextButton, row, 0);

      nextButton = new QPushButton(this);
      nextButton->setText("Delete Path");
      connect(nextButton, &QPushButton::clicked, [=] () { this->deletePath(spinBox->value()); });
      infoLayout->addWidget(nextButton, row, 1);

      nextButton = new QPushButton(this);
      nextButton->setText("Color Path");
      connect(nextButton, &QPushButton::clicked, [=] () { this->colorPath(spinBox->value()); });
      infoLayout->addWidget(nextButton, row, 2);
      
      ++row;
      nextButton = new QPushButton(this);
      nextButton->setText("Update Table");
      connect(nextButton, &QPushButton::clicked, [=] () { this->adjustDistanceTable(spinBox->value()); });
      infoLayout->addWidget(nextButton, row, 0);
      nextButton = new QPushButton(this);
      nextButton->setText("Set Plot Width");
      connect(nextButton, &QPushButton::clicked, [=] () { this->setPlotWidth(spinBox->value()); });
      infoLayout->addWidget(nextButton, row, 1);
      nextButton = new QPushButton(this);
      nextButton->setText("Set Font Size");
      connect(nextButton, &QPushButton::clicked, [=] () { this->setFontSize(spinBox->value()); });
      infoLayout->addWidget(nextButton, row, 2);
    }
    mpLayout->addLayout(infoLayout, 0, 1);

    setFontSize(mpContainer->fontSize);
  }

  /**
  */
  QuickPathAnalysisWindow::~QuickPathAnalysisWindow()
  {
  }

  /**
  */
  void QuickPathAnalysisWindow::open()
  {
    QRect rec = QApplication::desktop()->screenGeometry();
    resize(0.5*rec.width(), 0.5*rec.height());
    QDialog::open();
  }

  /**
  */
  void QuickPathAnalysisWindow::clearPaths()
  {
    auto N = mpPlot->graphCount();
    // zero is the safety distance graph
    for (auto& data : mpContainer->graphs)
    {
      mpPlot->removeGraph(data.graph);
      for (auto& item : data.lines)
        mpPlot->removeItem(item);
      for (auto& item : data.labels)
        mpPlot->removeItem(item);
    }
  }

  /** \brief adds a new set of graph data to the plot

    fills only the graph data
    add e.g. peaks via a call to #addPeaks()
  */
  void QuickPathAnalysisWindow::addDistances(const std::vector<double>& v)
  {
    const auto N = (int)v.size();
    QVector<double> xTime(N, 0);
    QVector<double> yDist(N, 0);
    for (int i(0); i<N; ++i)
    {
      xTime[i] = i;
      yDist[i] = v[i];
    }
    
    auto& data = mpContainer->addGraph();
    data.graph = mpPlot->addGraph();
    {
      // create and configure plottables:
      data.graph ->setData(xTime, yDist);
      data.graph ->rescaleAxes();
    }

    // resfresh safety distance
    auto key = mpContainer->safetyDistGraph->end->key();
    if (key < N)
      mpContainer->safetyDistGraph->end->setCoords(N, mpContainer->safetyDistGraph->end->value());
  }

  /**
  */
  void QuickPathAnalysisWindow::setPlotWidth(int idx)
  {
    for (auto& data : mpContainer->graphs)
    {
      auto pen = data.graph->pen();
      pen.setWidth(idx);
      data.graph->setPen(pen);
    }
    auto pen = mpContainer->safetyDistGraph->pen();
    pen.setWidth(idx);
    mpContainer->safetyDistGraph->setPen(pen);

    mpPlot->replot();
  }

  /**
  */
  void QuickPathAnalysisWindow::setFontSize(int idx)
  {
    mpContainer->fontSize = idx;

    auto font = mpPlot->xAxis->labelFont();
    font.setPointSize(idx);
    mpPlot->xAxis->setLabelFont(font);
    font = mpPlot->xAxis->tickLabelFont();
    font.setPointSize(idx);
    mpPlot->xAxis->setTickLabelFont(font);

    font = mpPlot->yAxis->labelFont();
    font.setPointSize(idx);
    mpPlot->yAxis->setLabelFont(font);
    font = mpPlot->yAxis->tickLabelFont();
    font.setPointSize(idx);
    mpPlot->yAxis->setTickLabelFont(font);

    font = mpContainer->safetyDistText->font();
    font.setPointSize(idx);
    mpContainer->safetyDistText->setFont(font);

    for (auto& data : mpContainer->graphs)
    {
      for (auto* label : data.labels)
      {
        auto pen = label->font();
        pen.setPointSize(idx);
        label->setFont(pen);
      }
    }
    mpPlot->replot();
  }

  /**
  */
  void QuickPathAnalysisWindow::setSafetyDistance(double d)
  {
    mpContainer->safetyDistGraph->start->setCoords(0, d);
    mpContainer->safetyDistGraph->end  ->setCoords(mpContainer->safetyDistGraph->end->key(), d);
  }

  /**
  */
  void QuickPathAnalysisWindow::addPeaks(const std::vector<ObstaclePeak>& peaks)
  {
    auto& data = mpContainer->graphs.back();
    for (const auto& tuple : peaks)
    {
      auto& key = std::get<0>(tuple);
      auto& idx = std::get<1>(tuple);
      auto& dist = std::get<2>(tuple);

      auto* pLine = new QCPItemLine(mpPlot);
      mpPlot->addItem(pLine);
      pLine->setClipAxisRect(mpPlot->axisRect());
      pLine->setClipToAxisRect(true);
      pLine->position("start")->setAxes(mpPlot->xAxis, mpPlot->yAxis);
      pLine->position("end")->setAxes(mpPlot->xAxis, mpPlot->yAxis);
      pLine->start->setCoords(idx, 0);
      pLine->end->setCoords(idx, dist);
      pLine->setHead(QCPLineEnding(QCPLineEnding::esDisc));
      data.lines.push_back(pLine);

      // add label for obstacle minimum:
      QCPItemText *pText = new QCPItemText(mpPlot);
      pText->setClipAxisRect(mpPlot->axisRect());
      pText->setClipToAxisRect(true);
      pText->position->setAxes(mpPlot->xAxis, mpPlot->yAxis);
      pText->setPositionAlignment(Qt::AlignHCenter | Qt::AlignBottom);
      pText->position->setCoords(idx, dist);
      pText->setText(key.c_str());
      pText->setTextAlignment(Qt::AlignBottom);
      pText->setPadding(QMargins(0, 0, 0, 10)); // thats the distance between "text and textbox"
      QFont font;
      pText->setFont(QFont(font.family(), mpContainer->fontSize));
      data.labels.push_back(pText);
    }
    auto idx = static_cast<int>( mpContainer->graphs.size() ) - 1;
    adjustDistanceTable(idx);
  }

  void QuickPathAnalysisWindow::deletePath(int idx)
  {
    if (mpContainer->graphs.size() <= idx)
      return;
    auto& data = mpContainer->graphs[idx];
    {
      mpPlot->removeGraph(data.graph);
      for (auto& item : data.lines)
        mpPlot->removeItem(item);
      for (auto& item : data.labels)
        mpPlot->removeItem(item);
    }
    mpContainer->graphs.erase(mpContainer->graphs.begin() + idx);
    compute();
  }

  void QuickPathAnalysisWindow::colorPath (int idx)
  {
    if (mpContainer->graphs.size() <= idx)
      return;
    auto& data = mpContainer->graphs[idx];
    {
      auto dialog = std::make_unique<QColorDialog>();
      dialog->exec();
      if (0 == dialog->result()) // cancel-button = 0, "x"-button = 0, ok-button = 1
        return;
      QColor qcolor = dialog->selectedColor();
      auto pen = QPen(qcolor);
      data.graph->setPen(pen);
    }
    mpPlot->replot();
  }

  /**
  */
  void QuickPathAnalysisWindow::adjustDistanceTable(int idx)
  {
    if (mpContainer->graphs.size() <= idx)
      return;

    for(int i(0); i<mpContainer->graphs.size(); ++i)
    {
      bool visible = idx == i;
      for (auto* item : mpContainer->graphs[i].labels)
        item->setVisible(visible);
      for (auto* item : mpContainer->graphs[i].lines)
        item->setVisible(visible);
    }
    mpPlot->replot();

    const auto& data = mpContainer->graphs[idx];
    //// legend
    const auto N_Peaks = static_cast<int>( data.lines.size() );
    auto* table = mpContainer->distanceTable;
    table->clear();
    table->setRowCount(N_Peaks +1);
    table->setWindowTitle("Minimal Distances");
    {
      auto* nextItem = new QTableWidgetItem();
      nextItem->setText("Object");
      table->setItem(0, 0, nextItem);
      nextItem = new QTableWidgetItem();
      nextItem->setText("Distance in mm");
      table->setItem(0, 1, nextItem);
    }
    // data
    LOG_LINE << "N_Peaks: " << N_Peaks;
    std::vector<int> ordering(N_Peaks);
    for (int i(0); i<ordering.size(); ++i)
    {
      ordering[i] = i;
    }
    sort(ordering.begin(), ordering.end(), [&] (int lhs, int rhs)
    {
      auto keyL = data.labels[lhs]->text().toStdString();
      std::transform(keyL.begin(), keyL.end(), keyL.begin(), ::tolower);
      auto keyR = data.labels[rhs]->text().toStdString();
      std::transform(keyR.begin(), keyR.end(), keyR.begin(), ::tolower);
      return std::lexicographical_compare(keyL.begin(), keyL.end(), keyR.begin(), keyR.end());
    });

    for (int j(0); j<(int)ordering.size(); ++j)
    {
      int i = ordering[j];
      auto& key  = data.labels[i]->text();
      auto& idx  = data.lines[i]->start->coords().rx();
      auto& dist = data.lines[i]->end->coords().ry();
      LOG_LINE << key.toStdString() << ": " << idx << " -> " << dist;
      auto* nextItem = new QTableWidgetItem();
      nextItem->setText(key);
      table->setItem(i+1, 0, nextItem);
      nextItem = new QTableWidgetItem();
      nextItem->setText(std::to_string(dist).c_str());
      table->setItem(i+1, 1, nextItem);
    }
  }
  
  /**
  */
  void QuickPathAnalysisWindow::compute()
  {
    double ymin(0);
    double ymax(1);
    int xmin(0);
    int xmax(0);
    for (const auto& data : mpContainer->graphs)
    {
      if (!data.graph->data()->empty())
      {
        for (auto iter = data.graph->data()->begin(); iter != data.graph->data()->end(); ++iter)
        {
          ymin = std::min(ymin, iter.value().value);
          ymax = std::max(ymax, iter.value().value);
        }
        
        const auto& val = data.graph->data()->lastKey();
        xmin = std::max(xmin, static_cast<int>(0.15*val) );
        xmax = std::max(xmax, static_cast<int>(1.15*val) );
      }
    }
    auto safetyDistance = static_cast<double>( mpContainer->safetyDistGraph->start->coords().ry() );
    ymax = std::max(ymax, safetyDistance) + 0.1*(ymax - ymin);
    ymin = std::min(-0.1, ymin);
    mpPlot->yAxis->setRange(ymin, ymax);

    xmin *= -1;
    mpPlot->xAxis->setRange(xmin, xmax);
    mpPlot->replot();
  }

  /**
  */
  void QuickPathAnalysisWindow::printPlot(const std::string& filename)
  {
    /*namespace fs = boost::filesystem;
    const auto input = fs::path(filename);
    const auto fnPng = input.parent_path() / (input.stem().string() + ".png");
    mpPlot->savePng(fnPng.string().c_str());

    const auto fnPrinter = input.parent_path() / (input.stem().string() + ".pdf");
    QPrinter printer(QPrinter::HighResolution);
    printer.setOutputFormat(QPrinter::PdfFormat);
    printer.setOutputFileName(fnPrinter.string().c_str());
    printer.setPageMargins(12, 16, 12, 20, QPrinter::Millimeter);
    printer.setFullPage(false);

    QPainter painter(&printer);

    double xscale = printer.pageRect().width() / double(this->width());
    double yscale = printer.pageRect().height() / double(this->height());
    double scale = qMin(xscale, yscale);
    painter.translate(printer.paperRect().center());
    painter.scale(scale, scale);
    painter.translate(-this->width()/ 2, -this->height()/ 2);
    this->render(&painter);*/
  }
}
}