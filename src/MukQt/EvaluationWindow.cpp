#include "private/muk.pch"
#include "EvaluationWindow.h"

#include <qpushbutton.h>
#include <qcolor.h>

#include <boost/format.hpp>

namespace gris
{
namespace muk
{
  /**
  */
  EvaluationWindow::EvaluationWindow(QWidget* parent)
    : QWidget(parent)
    , mPlotIdx(0)
  {
    this->setObjectName("EvaluationWindow");
    this->setStyleSheet("background-color:gray;");
    mpLayout = new QGridLayout(this);
    auto button = new QPushButton(this);
    button->setText("Next Plot");
    connect(button, &QPushButton::clicked, this, &EvaluationWindow::showNextPlot);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    button->setStyleSheet( (boost::format("background-color: rgb(%d,%d,%d);") % 0 % 255 % 255).str().c_str() );
    mpLayout->addWidget(button, 0, 0);
  }

  /**
  */
  EvaluationWindow::~EvaluationWindow()
  {
    for(auto& pWidget : mPlots)
      mpLayout->removeWidget(pWidget.get());
  }

  /**
  */
  void EvaluationWindow::addWidget(QWidget* w, int row, int column, int rowSpan, int colSpan)
  {
    w->setParent(this);
    mpLayout->addWidget(w, row, column, rowSpan, colSpan);    
  }

  /**
  */
  void EvaluationWindow::showNextPlot()
  {    
    if (mPlots.empty())
      return;
    mpLayout->removeWidget(mPlots[mPlotIdx].get());
    ++mPlotIdx;
    if(mPlotIdx==mPlots.size())
      mPlotIdx = 0;
    mpLayout->addWidget(mPlots[mPlotIdx].get(), 1, 0);
    for (size_t i(0); i<mPlots.size(); ++i)
      mPlots[i]->setVisible(i == mPlotIdx);
    LOG_LINE << "plot " << mPlotIdx << " of " <<  mPlots.size();
  }

  /**
  */
  void EvaluationWindow::addPlot(std::shared_ptr<QWidget> w)
  {
    w->setParent(this);
    w->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    mPlots.push_back(w);
  }

  /**
  */
  void EvaluationWindow::clearPlots()
  {
    for(auto& pWidget : mPlots)
      mpLayout->removeWidget(pWidget.get());
    mPlots.clear();
  }

}
}