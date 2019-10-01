#pragma once

#include "muk_qt_api.h"

#include <QMenuBar>

#include <qdialog.h>

QT_BEGIN_NAMESPACE
class QLineEdit;
class QDialogButtonBox;
QT_END_NAMESPACE

namespace gris
{
  namespace muk
  {
    /** \brief A menu bar for the application "MuknoPlanner.exe"
    */
    class MUK_QT_API MukQMenuBar : public QMenuBar
    {
      Q_OBJECT

      public:
        explicit MukQMenuBar(QWidget *parent = 0);
        ~MukQMenuBar();

      signals:
        void loadSceneClicked(const std::string& filename);
        void savePathClicked(const std::string& filename);
        void loadPathClicked(const std::string& filename);
        void loadAsObstacle(const char* );

        void quickPlanningConfigRequested();

        void setDefaultFocusClicked();

      public:
        void setLastSceneFiles(const std::list<std::string>& v);
        
      private slots:
        void actionSavePath();
        void actionLoadPath();
        void actionLoadAsObstacle();

        void actionShowConsole();
        void actionSetDefaultFocus();
        void actionComputeKappa();

        void actionAbout();

      private:
        void createFileMenu();
        void setOrderOfMenus();

      private:
        QMenu* mFileMenu;
        QMenu* mQuickConfigureMenu;
        QMenu* mOptionsMenu;
        QMenu* mHelpMenu;

        const std::list<std::string>* mLastSceneFiles;
    };

    /**
    */
    class MultipleInputDialog : public QDialog
    {
      Q_OBJECT

      public:
        MultipleInputDialog();

      private:
        void computeAndShowKappa();

      private:
        QLineEdit* mLineEditAngle;
        QLineEdit* mLineEditLength;
        QLineEdit* mLineEditKappa;
        QDialogButtonBox* mButtonBox;
    };

  }
}
