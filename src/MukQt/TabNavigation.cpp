#include "private/muk.pch"
#include "TabNavigation.h"

#include "MukCommon/NavigatorFactory.h"
#include "MukCommon/CalibrationFactory.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/MukException.h"
#include "MukCommon/overload_deduction.h"

#include <QtWidgets/QLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QAction>

#include <iostream>

namespace gris
{
  namespace muk
  {
    /** \brief The Widget that holds all interaction for Navigation

    Navigator Label + Combobox
    Calibrator Label + Combobox
    | verticalStretch
    Buttons:
    - init
    - calibrate
    - start
    - stop
    - proceed
    | verticalStretch
    StatusText

    */
    TabNavigation::TabNavigation(QWidget *parent)
      : QWidget(parent), mInitialized(false), mRunning(false)
    {
      this->setObjectName("TabNavigation");
      {
        QSizePolicy sizePolicy;
        sizePolicy.setHeightForWidth(this->sizePolicy().hasHeightForWidth());
        this->setSizePolicy(sizePolicy);
        this->setMaximumSize(QSize(16777215, 16777215));
      }

      // major layout
      auto mainLayout = std::make_unique<QVBoxLayout>(this);
      mainLayout->setObjectName(QStringLiteral("mainLayout"));

      auto selectorLayoutBox = new QGridLayout();
      selectorLayoutBox->setObjectName(QStringLiteral("selectorLayoutBox"));
      {
        auto navLabel = new QLabel(this);
        auto calibLabel = new QLabel(this);
        mpNavigatorSelection = new QComboBox(this);
        mpCalibratorSelection = new QComboBox(this);
        {
          navLabel->setObjectName(QStringLiteral("labelNavigator"));
          mpNavigatorSelection->setObjectName(QStringLiteral("comboBoxNavigators"));
          calibLabel->setObjectName(QStringLiteral("labelCalibrator"));
          mpCalibratorSelection->setObjectName(QStringLiteral("comboBoxCalibrators"));
          {
            QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            sizePolicy.setHorizontalStretch(0);
            sizePolicy.setVerticalStretch(0);
            sizePolicy.setHeightForWidth(mpNavigatorSelection->sizePolicy().hasHeightForWidth());
            mpNavigatorSelection->setSizePolicy(sizePolicy);
            sizePolicy.setHeightForWidth(mpCalibratorSelection->sizePolicy().hasHeightForWidth());
            mpCalibratorSelection->setSizePolicy(sizePolicy);
           }
          selectorLayoutBox->addWidget(navLabel, 0, 0, 1, 1);
          selectorLayoutBox->addWidget(mpNavigatorSelection, 0, 1, 1, 1);
          selectorLayoutBox->addWidget(calibLabel, 1, 0, 1, 1);
          selectorLayoutBox->addWidget(mpCalibratorSelection, 1, 1, 1, 1);
        }
      }
      mainLayout->addLayout(selectorLayoutBox);
      auto topSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Preferred);
      mainLayout->addItem(topSpacer);

      auto buttonLayoutBox = new QGridLayout();
      buttonLayoutBox->setObjectName("buttonLayoutBox");
      {
        mpInitButton = new QPushButton(this);
        mpInitButton->setObjectName("initButton");
        buttonLayoutBox->addWidget(mpInitButton, 0, 0, 1, 1);
        mpCalibrateButton = new QPushButton(this);
        mpCalibrateButton->setObjectName("calibrateButton");
        mpCalibrateButton->setEnabled(false);
        buttonLayoutBox->addWidget(mpCalibrateButton, 1, 0, 1, 1);
        mpStartButton = new QPushButton(this);
        mpStartButton->setObjectName("startButton");
        mpStartButton->setEnabled(false);
        buttonLayoutBox->addWidget(mpStartButton, 2, 0, 1, 1);
        mpStopButton = new QPushButton(this);
        mpStopButton->setObjectName("stopButton");
        mpStopButton->setEnabled(false);
        buttonLayoutBox->addWidget(mpStopButton, 3, 0, 1, 1);
        mpProceedButton = new QPushButton(this);
        mpProceedButton->setObjectName("proceedButton");
        mpProceedButton->setEnabled(false);
        buttonLayoutBox->addWidget(mpProceedButton, 4, 0, 1, 1);
      }
      mainLayout->addLayout(buttonLayoutBox);

      mainLayout->addStretch(1);

      auto infoLayoutBox = new QGridLayout();
      {
        auto statusTextLabel = new QLabel(this);
        statusTextLabel->setObjectName("labelStatus");
        infoLayoutBox->addWidget(statusTextLabel, 0, 0, 1, 1);
        mpStatusText = new QLabel(this);
        mpStatusText->setObjectName("labelStatusText");
        mpStatusText->setWordWrap(true);
        mpStatusText->setMinimumHeight(64);
        infoLayoutBox->addWidget(mpStatusText, 0, 1, 1, 1);
      }
      mainLayout->addLayout(infoLayoutBox);

      this->setLayout(mainLayout.release());

      setupConnections();
    }

    /**
    */
    TabNavigation::~TabNavigation()
    {
      // all child components are automatifcally destroyed
    }

    /**
    */
    void TabNavigation::initialize(const std::shared_ptr<MukScene> pScene)
    {
      { // fill the Navigator ComboBox with the Keys
        auto* pCombobox = this->findChild<QComboBox*>("comboBoxNavigators");
        const QSignalBlocker blocker(pCombobox);

        pCombobox->clear();
        std::vector<std::string> keys;
        GetNavigatorFactory().getKeys(keys);
        std::for_each(keys.begin(), keys.end(), [&](const std::string& key) { pCombobox->addItem(key.c_str()); });
        int index = pCombobox->findText(pScene->getNavigation()->getNavigatorName().c_str());
        if (index < 0)
          throw MUK_EXCEPTION("Could not find navigator", pScene->getNavigation()->getNavigatorName().c_str());
        pCombobox->setCurrentIndex(index);
      }

      { // fill the Calibrator ComboBox with the Keys
        auto* pCombobox = this->findChild<QComboBox*>("comboBoxCalibrators");
        const QSignalBlocker blocker(pCombobox);

        pCombobox->clear();
        std::vector<std::string> keys;
        GetCalibrationFactory().getKeys(keys);
        std::for_each(keys.begin(), keys.end(), [&](const std::string& key) { pCombobox->addItem(key.c_str()); });
/*        int index = pCombobox->findText(pScene->getNavigation()->getNavigatorName().c_str());
        if (index < 0)
          throw MUK_EXCEPTION("Could not find navigator", pScene->getNavigation()->getNavigatorName().c_str());
        pCombobox->setCurrentIndex(index);
*/      }
    }

    /**
    */
    void TabNavigation::retranslateUi(QMainWindow *MainWindow)
    {
      this->findChild<QLabel*>("labelNavigator")->setText(QApplication::translate("MainWindow", "Navigator", 0));
      this->findChild<QLabel*>("labelCalibrator")->setText(QApplication::translate("MainWindow", "Calibrator", 0));
      mpCalibrateButton->setText(QApplication::translate("MainWindow", "Calibrate", 0));
      mpInitButton->setText(QApplication::translate("MainWindow", "Initialize", 0));
      mpStartButton->setText(QApplication::translate("MainWindow", "Start", 0));
      mpStopButton->setText(QApplication::translate("MainWindow", "Stop", 0));
      mpProceedButton->setText(QApplication::translate("MainWindow", "Proceed", 0));
      this->findChild<QLabel*>("labelStatus")->setText(QApplication::translate("MainWindow", "Status", 0));
    }

    /**
    */
    void TabNavigation::setupConnections()
    {
      connect(mpInitButton,    &QPushButton::clicked, this, &TabNavigation::initRequested);
      connect(mpStopButton,    &QPushButton::clicked, this, &TabNavigation::stopRequested);
      connect(mpProceedButton, &QPushButton::clicked, this, &TabNavigation::proceedRequested);
      connect(mpStartButton,   &QPushButton::clicked, this, &TabNavigation::startRequested);
      connect(mpCalibrateButton, &QPushButton::clicked, this, &TabNavigation::calibrationRequested);

      connect(mpNavigatorSelection, &QComboBox::currentTextChanged, [&](const QString& str) { emit navigatorChanged(str.toLocal8Bit().constData()); });
      connect(mpCalibratorSelection, &QComboBox::currentTextChanged, [&](const QString& str) { emit calibratorChanged(str.toLocal8Bit().constData()); });
    }

    /**
    */
    void TabNavigation::showMessageDialog(const std::string& msg, const bool waitProceed)
    {
      QMessageBox infobox;
      infobox.setText(msg.c_str());
      LOG_LINE << "Message from Navigation:\n" << msg;
      infobox.setIcon(QMessageBox::Information);
      infobox.setStandardButtons(QMessageBox::Ok);
      infobox.setWindowTitle("Message from Navigation");
      infobox.exec();
      if (waitProceed)
        emit proceedRequested();
    }

    /**
    */
    void TabNavigation::showStatusMessage(const std::string& msg)
    {
      mpStatusText->setText(QString::fromStdString(msg));
    }

    /**
    */
    void TabNavigation::logMessage(const std::string& msg)
    {
      LOG_LINE << msg;
    }

    /**
    */
    void TabNavigation::toggleActivationInitialization(const bool & state)
    {
      if (state == mInitialized) return;
      mpStartButton->setEnabled(state && !mRunning);
      mpCalibrateButton->setEnabled(mCalibratorValid && state);
      mpProceedButton->setEnabled(state && !mRunning);
      mInitialized = state;
    }

    /**
    */
    void TabNavigation::toggleActivationRunning(const bool & state)
    {
      if (state == mRunning) return;
      mpStartButton->setEnabled(!state && mInitialized);
      mpStopButton->setEnabled(state);
      mpInitButton->setEnabled(!state);
      mpProceedButton->setEnabled(!state && mInitialized);
      mRunning = state;
    }
    void TabNavigation::toggleActivationCalibratorValid(const bool & state)
    {
      if (mCalibratorValid == state) return;
      mpCalibrateButton->setEnabled(mInitialized && state);
      mCalibratorValid = state;
    }
  }
}

