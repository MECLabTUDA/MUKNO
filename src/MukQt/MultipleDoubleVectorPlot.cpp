#include "private/muk.pch"
#include "MultipleDoubleVectorPlot.h"

#include "MukCommon/MukException.h"

#include <qcustomplot.h>

#include <boost/format.hpp>


namespace gris
{
namespace muk
{
  /**
  */
  MultipleDoubleVectorPlot::MultipleDoubleVectorPlot(QWidget* parent)
    : MukPlot(parent)
  {
    //mpPlot->plotLayout()->clear(); // clear default axis rect so we can start from scratch
    //QCPAxisRect *axisRect = new QCPAxisRect(mpPlot);
    //{
    //  axisRect->setupFullAxesBox(true);
    //  axisRect->axis(QCPAxis::atRight, 0)->setTickLabels(true);      
    //}
    //QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    //mpPlot->plotLayout()->addElement(0, 0, axisRect);
    mpPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);    
  }

  /**
  */
  MultipleDoubleVectorPlot::~MultipleDoubleVectorPlot()
  {
  }

  /**
  */
  void MultipleDoubleVectorPlot::setNumberOfGraphs(size_t N)
  {
    mpPlot->clearGraphs();
    xmax.resize(N);
    ymax.resize(N);
    for (size_t i(0); i<N; ++i)
    {
      QCPGraph *graph = mpPlot->addGraph(mpPlot->axisRect()->axis(QCPAxis::atBottom), mpPlot->axisRect()->axis(QCPAxis::atLeft));      
      switch (i)
      {
        case 0 :
          graph->setPen(QPen(Qt::blue));
          break;
        case 1: 
          graph->setPen(QPen(Qt::red));
          break;
        default:
          graph->setPen(QPen(Qt::green));
      }      
    }
  }

  /**
  */
  void MultipleDoubleVectorPlot::setData(size_t i, const std::vector<double>& data)
  {
    const auto N = static_cast<unsigned int>(data.size()); // x64 qt uses unsigned int?
    // create data axis x,y,
    QVector<double> xTime(N, 0);
    QVector<double> yVal(N);
    for (unsigned int k(0); k<N; ++k)
    {
      xTime[k] = k;
      yVal[k] = data[k];
    }
    //
    QCPGraph *graph = mpPlot->graph(static_cast<int>(i));
    graph->setData(xTime, yVal);
    graph->rescaleAxes();
    xmax[i] = data.size();
    ymax[i] = data.empty() ? 0 : *std::max_element(data.begin(), data.end());
    mpPlot->axisRect()->axis(QCPAxis::atLeft, 0)->setRangeLower(0);
  }

  /**
  */
  void MultipleDoubleVectorPlot::setData(const std::vector<std::vector<double>>& data)
  {
    const size_t N = data.size();    
    setNumberOfGraphs(N);
    for (size_t i(0); i<N; ++i)
    {
      setData(i, data[i]);
    }
  }

  void MultipleDoubleVectorPlot::setGraphTitle(int i, const std::string& graphTitle)
  {
    if (i >= mpPlot->graphCount())
    {
      auto str = (boost::format("index bigger than number of graphs: %d >= %d") % i % mpPlot->graphCount() ).str();
      throw MUK_EXCEPTION_SIMPLE(str.c_str());
    }
    mpPlot->graph(i)->setName(graphTitle.c_str());
    mpPlot->legend->setVisible(true);
  }

  void MultipleDoubleVectorPlot::setXLabel(const char* str)
  {
    mpPlot->xAxis->setLabel(str);
  }

  void MultipleDoubleVectorPlot::setYLabel(const char* str)
  {
    mpPlot->yAxis->setLabel(str);
  }

  void MultipleDoubleVectorPlot::adjust()
  {
    if (!xmax.empty())
    {
      mpPlot->axisRect()->axis(QCPAxis::atLeft, 0)->setRangeUpper(1.25 * *std::max_element(ymax.begin(), ymax.end()));
      mpPlot->axisRect()->axis(QCPAxis::atLeft, 0)->setRangeLower(-0.05 * *std::max_element(ymax.begin(), ymax.end()));
            
      mpPlot->axisRect()->axis(QCPAxis::atBottom, 0)->setRangeUpper(1.1 * *std::max_element(xmax.begin(), xmax.end()));
      mpPlot->axisRect()->axis(QCPAxis::atBottom, 0)->setRangeLower(-5);      
    }
  }

}
}