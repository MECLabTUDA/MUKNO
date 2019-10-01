#include "private/muk.pch"
#include "BoxPlots.h"

#include "MukCommon/MukException.h"

#include <qcustomplot.h>

#include <boost/format.hpp>

#include <string>

namespace gris
{
namespace muk
{
  /**
  */
  BoxPlots::BoxPlots(QWidget* parent)
    : MukPlot(parent)
  {
    mpPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom); // | QCP::iSelectPlottables);
    // no default text at x axis
    mpPlot->xAxis->setAutoTicks(false);
    mpPlot->xAxis->setAutoTickLabels(false);
        
    mpPlot->xAxis->setTickVector(QVector<qreal>());    
    mpPlot->xAxis->setTickVectorLabels(QVector<QString>());
    
    mpPlot->xAxis->setSubTickCount(0);
    mpPlot->xAxis->setTickLength(0, 4);
    mpPlot->xAxis->scaleRange(1.7, mpPlot->xAxis->range().center());
    mpPlot->yAxis->setRange(0, 0.1);
  }

  /**
  */
  BoxPlots::~BoxPlots()
  {
  }

  /**
  */
  void BoxPlots::addBox(const std::vector<double>& data)
  {
    QCPStatisticalBox* sample = new QCPStatisticalBox(mpPlot->xAxis, mpPlot->yAxis);
    QBrush boxBrush(QColor(60, 60, 255, 100));
    sample->setBrush(boxBrush);
    mpPlot->addPlottable(sample);
    const size_t N = data.size();
    std::vector<double> work(N);
    std::copy(data.begin(), data.end(), work.begin());
    std::sort(work.begin(), work.end());
    sample->setKey(mpPlot->plottableCount()); // start at position x = 1
    if (work.empty())
    {
      sample->setMinimum(0);
      sample->setLowerQuartile(0);
      sample->setMedian(0);
      sample->setUpperQuartile(0);
      sample->setMaximum(0);
    }
    else
    {
      sample->setMinimum( work.front() );
      sample->setMaximum( work.back() );
      auto median = work[std::min(work.size()-1, (size_t) std::round(0.5*work.size()))];
      sample->setMedian( median );
      auto lowerq = work[std::min(work.size()-1, (size_t) std::round(0.25*work.size()))];
      sample->setLowerQuartile( lowerq );
      auto upperq = work[std::min(work.size()-1, (size_t) std::round(0.75*work.size()))];
      sample->setUpperQuartile( upperq );
    }
    auto ticks = mpPlot->xAxis->tickVector();
    ticks.push_back(mpPlot->plottableCount());
    mpPlot->xAxis->setTickVector(ticks);
    auto tickLabels = mpPlot->xAxis->tickVectorLabels();
    tickLabels.push_back(std::to_string(mpPlot->plottableCount()).c_str());
    mpPlot->xAxis->setTickVectorLabels(tickLabels);
  }

  /**
  */
  void BoxPlots::setSampleName(int i, const char* str)
  {    
    if (i >= mpPlot->plottableCount())
    {
      auto str = (boost::format("index bigger than number of graphs: %d >= %d") % i % mpPlot->graphCount() ).str();
      throw MUK_EXCEPTION_SIMPLE(str.c_str());
    }
    auto ticks = mpPlot->xAxis->tickVector();
    ticks[i] = i+1; // at position x = i+1 (remember, we start painting boxes at x = 1)    
    mpPlot->xAxis->setTickVector(ticks);
    auto tickLabels = mpPlot->xAxis->tickVectorLabels();
    tickLabels[i] = str;
    mpPlot->xAxis->setTickVectorLabels(tickLabels);
    mpPlot->xAxis->setLabelFont(QFont("Helvetica", 12));
  }

  /** \brief sets the rotation of the x label
    
    unit: degress
    orientation: clockwise rotation
  */
  void BoxPlots::setSampleRotation(double val)
  {
    mpPlot->xAxis->setTickLabelRotation(val);
  }

  /**
  */
  void BoxPlots::setYLabel(const char* str)
  {
    mpPlot->yAxis->setLabel(str);
    mpPlot->yAxis->setLabelFont(QFont("Helvetica", 12));
    mpPlot->yAxis->setTickLabelFont(QFont("Helvetica", 12));
  }

  /**
  */
  void BoxPlots::adjust()
  {
    mpPlot->rescaleAxes();
  }
}
}
