#include "private/muk.pch"
#include "QuickPlanningConfigurationWindow.h"

#include "MukCommon/MukException.h"

#include <qlayout.h>
#include <qpushbutton.h>
#include <QSpacerItem>
#include <qlabel.h>
#include <qlineedit.h>
#include <qcombobox.h>

namespace gris
{
namespace muk
{
  /**
  */
  QuickPlanningConfigurationWindow::QuickPlanningConfigurationWindow()
    : QDialog()
  {
    auto* pMainLayout = new QVBoxLayout(this);
    auto* pOptionsLayout = new QGridLayout();
    {
      auto* nextLabel      = new QLabel(this);
      nextLabel->setText("Planner:");
      pOptionsLayout->addWidget(nextLabel, 0,0);
      mPlanner = new QComboBox(this);
      for (const auto& obj : mConfigs)
      {
        mPlanner->addItem(obj.getPlanner().c_str());
      }
      connect(mPlanner, &QComboBox::currentTextChanged, this, &QuickPlanningConfigurationWindow::plannerSelected);
      pOptionsLayout->addWidget(mPlanner, 0,1);

      nextLabel      = new QLabel(this);
      nextLabel->setText("Pruner:");
      pOptionsLayout->addWidget(nextLabel, 1,0);
      mPruner = new QLineEdit(this);
      mPruner->setReadOnly(true);
      pOptionsLayout->addWidget(mPruner, 1,1);

      nextLabel      = new QLabel(this);
      nextLabel->setText("Interpolator:");
      pOptionsLayout->addWidget(nextLabel, 2,0);
      mInterpolator = new QLineEdit(this);
      mInterpolator->setReadOnly(true);
      pOptionsLayout->addWidget(mInterpolator, 2,1);

      nextLabel      = new QLabel(this);
      nextLabel->setText("Resolution:");
      pOptionsLayout->addWidget(nextLabel, 3,0);
      mSphereResolution = new QSpinBox(this);
      mSphereResolution->setMinimum(0);
      mSphereResolution->setMaximum(200);
      pOptionsLayout->addWidget(mSphereResolution, 3,1);

      nextLabel      = new QLabel(this);
      nextLabel->setText("New Paths to Create:");
      pOptionsLayout->addWidget(nextLabel, 4,0);
      mNewPaths = new QSpinBox(this);
      mNewPaths->setMinimum(1);
      mNewPaths->setMaximum(1000);
      pOptionsLayout->addWidget(mNewPaths, 4,1);

      nextLabel      = new QLabel(this);
      nextLabel->setText("calculation time:");
      pOptionsLayout->addWidget(nextLabel, 5,0);
      mCalcTime = new QDoubleSpinBox(this);
      mCalcTime->setMinimum(0.01);
      mCalcTime->setSingleStep(0.1);
      mCalcTime->setMaximum(10);
      pOptionsLayout->addWidget(mCalcTime, 5,1);

      nextLabel      = new QLabel(this);
      nextLabel->setText("step size:");
      pOptionsLayout->addWidget(nextLabel, 6,0);
      mStepSize = new QDoubleSpinBox(this);
      mStepSize->setMinimum(0.1);
      mStepSize->setSingleStep(0.1);
      mStepSize->setMaximum(10);
      pOptionsLayout->addWidget(mStepSize, 6,1);
    }
    pMainLayout->addLayout(pOptionsLayout);

    auto* spcaer = new QSpacerItem(0,0, QSizePolicy::Minimum, QSizePolicy::Minimum);
    pMainLayout->addItem(spcaer);

    auto* pFinishLayout = new QHBoxLayout();
    {
      auto* pSpacer = new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum);
      pFinishLayout->addItem(pSpacer);
      auto* pOkButton = new QPushButton(this);
      pOkButton->setText("Configure");
      pFinishLayout->addWidget(pOkButton);
      connect(pOkButton, &QPushButton::clicked, this, &QDialog::accept);

      auto* pCancelButton = new QPushButton(this);
      pCancelButton->setText("Cancel");
      connect(pCancelButton, &QPushButton::clicked, this, &QDialog::reject);
      pFinishLayout->addWidget(pCancelButton);
    }
    pMainLayout->addLayout(pFinishLayout);
  }

  /**
  */
  void QuickPlanningConfigurationWindow::addConfig(const QuickPlanningConfigurator& config)
  {
    mConfigs.push_back(config);
  }

  /**
  */
  void QuickPlanningConfigurationWindow::plannerSelected(const QString& qstr)
  {
    const std::string str = qstr.toLocal8Bit().constData();
    auto iter = std::find_if(mConfigs.begin(), mConfigs.end(), [&] (const QuickPlanningConfigurator& config) { return str == config.getPlanner(); });
    mPruner->setText(iter->getPruner().c_str());
    mInterpolator->setText(iter->getInterpolator().c_str());
    mSphereResolution->setValue((int)iter->getSphereResolution());
    mNewPaths->setValue((int)iter->getNewPaths());
    mCalcTime->setValue(iter->getCalcTime());
    mStepSize->setValue(iter->getStepSize());
  }

  /**
  */
  std::string QuickPlanningConfigurationWindow::getPlanner() const
  {
    return mPlanner->currentText().toLocal8Bit().constData();
  }

  /**
  */
  std::string QuickPlanningConfigurationWindow::getInterpolator() const
  {
    return mInterpolator->text().toLocal8Bit().constData();
  }

  /**
  */
  std::string QuickPlanningConfigurationWindow::getPruner() const
  {
    return mPruner->text().toLocal8Bit().constData();
  }

  /**
  */
  size_t QuickPlanningConfigurationWindow::getSphereResolution() const
  {
    return mSphereResolution->value();
  }

  /**
  */
  size_t QuickPlanningConfigurationWindow::getNewPaths() const
  {
    return mNewPaths->value();
  }

  /**
  */
  void QuickPlanningConfigurationWindow::reloadConfigs()
  {
    if (mConfigs.empty())
    {
      throw MUK_EXCEPTION_SIMPLE("No default configurations available");
    }
    {
      QSignalBlocker block(mPlanner);
      mPlanner->clear();
      for(const auto& config: mConfigs)
        mPlanner->addItem(config.getPlanner().c_str());
    }
    plannerSelected(mConfigs.back().getPlanner().c_str());
  }

  /**
  */
  double QuickPlanningConfigurationWindow::getCalcTime() const 
  {
    return mCalcTime->value();
  }

  /**
  */
  double QuickPlanningConfigurationWindow::getStepSize() const
  {
    return mStepSize->value();
  }
}
}