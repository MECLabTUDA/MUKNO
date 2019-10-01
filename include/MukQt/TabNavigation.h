#pragma once

#include "muk_qt_api.h"

#include "MukCommon/MukScene.h"

#include <QtWidgets/QPushButton>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QLabel>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_QT_API TabNavigation : public QWidget
    {
      Q_OBJECT

      public:
        explicit TabNavigation(QWidget *parent = 0);
        ~TabNavigation();

      signals:
        void calibratorChanged(const std::string&);
        void navigatorChanged(const std::string&);
        void calibrationRequested();
        void initRequested();
        void startRequested();
        void stopRequested();
        void proceedRequested();

        // slots for changing data
      public slots:
        void showMessageDialog(const std::string& msg, const bool waitProceed);
        void showStatusMessage(const std::string& msg);
        void logMessage(const std::string& msg);
        void toggleActivationInitialization(const bool& state);
        void toggleActivationRunning(const bool& state);
        void toggleActivationCalibratorValid(const bool& state);

      public: 
        void initialize(const std::shared_ptr<MukScene> pscene);

      public:
        void retranslateUi(QMainWindow *MainWindow);
        void setupConnections();

      private:
        QPushButton* mpCalibrateButton;
        QPushButton* mpStartButton;
        QPushButton* mpInitButton;
        QPushButton* mpStopButton;
        QPushButton* mpProceedButton;
        QComboBox*   mpCalibratorSelection;
        QComboBox*   mpNavigatorSelection;
        QLabel*      mpStatusText;

        bool mInitialized, mRunning, mCalibratorValid;
    };
  }
}
