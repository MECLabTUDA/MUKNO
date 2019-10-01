#include "private/muk.pch"
#include "MukPlot.h"

#include <qtoolbar.h>

#include <qcustomplot.h>

#include <memory>

namespace gris
{
  namespace muk
  {
    /**
    */
    MukPlot::MukPlot(QWidget* parent)
      : QWidget(parent)
      , mTitleSize(20)
      , mLabelSize(20)
      , mTickSize(20)
    {
      this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

      auto* layout = new QVBoxLayout(this);
      {
        auto* toolbar = new QToolBar(this);
        auto* saveAction = new QAction("Save", toolbar);
        connect(saveAction, &QAction::triggered, this, &MukPlot::save);
        toolbar->addAction(saveAction);
        auto* toggleAction = new QAction("Toggle", toolbar);
        connect(toggleAction, &QAction::triggered, this, &MukPlot::toggle);
        toolbar->addAction(toggleAction);
        layout->addWidget(toolbar);        
      }
      {
        mpPlot = new QCustomPlot(this);
        mpPlot->setSizePolicy(this->sizePolicy());
        layout->addWidget(mpPlot);
      }
    }

    /**
    */
    MukPlot::~MukPlot()
    {
    }

    /**
    */
    void MukPlot::save()
    {
      QString qFile = QFileDialog::getSaveFileName(this, tr("Save File"), QDir::currentPath(), tr("Png (*.png)"));
      if (qFile.isEmpty())
        return;
      mpPlot->savePng(qFile.toLocal8Bit().constData());
    }

    /**
    */
    void MukPlot::setTitle(const char* str)
    {
      mpPlot->plotLayout()->insertRow(0);
      if (mpPlot->plotLayout()->hasElement(0, 0))
      {
        auto* pTitle = dynamic_cast<QCPPlotTitle*>(mpPlot->plotLayout()->element(0, 0));
        pTitle->setText(str);
      }
      else
        mpPlot->plotLayout()->addElement(0, 0, new QCPPlotTitle(mpPlot, str));
    }

    /**
    */
    void MukPlot::setTitleSize(unsigned int size)
    {
      mTitleSize = size;
      if (!mpPlot->plotLayout()->hasElement(0, 0))
      {
        mpPlot->plotLayout()->addElement(0, 0, new QCPPlotTitle(mpPlot, "no title"));
      }
      auto* pTitle = dynamic_cast<QCPPlotTitle*>(mpPlot->plotLayout()->element(0, 0));
      pTitle->setFont(QFont(mFontName.c_str(), mTitleSize));
    }

    /**
    */
    void MukPlot::toggle()
    {
//      auto dummy =  std::make_unique<
      auto dialog = std::make_unique<QDialog>();
      auto layout = new QGridLayout();
      layout->addWidget(this, 0,0);
      dialog->setLayout(layout);
      QSize size(500, 300);
      dialog->resize(size);
      dialog->exec();
      layout->removeWidget(this);      
    }
  }
}