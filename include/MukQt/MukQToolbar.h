#pragma once

#include "muk_qt_api.h"

#include <QToolBar>

namespace gris
{
  namespace muk
  {

    class MUK_QT_API MukQToolBar : public QToolBar
    {
      Q_OBJECT

      public:
        MukQToolBar(QWidget *parent = 0);
        ~MukQToolBar();

      public:
        void setupConnections();
        void setLastSceneDir(const std::string& dir) { mLastSceneDir = dir; };

      signals:
        void createNewSceneClicked();
        void saveSceneClicked(const char* filename);
        void loadSceneClicked(const char* filename);
        void loadObstacleFileClicked(const std::string& filename);
		    void loadCTFileClicked(const std::string& filename);
		    void loadSegmentationFileClicked(const std::string& filename);
        //void loadFiles(size_t n, const char** filenames);
        void focusClicked();
		    void focusOnWaypointClicked();
        void pathAnalysisRequested();

      private slots:
        void actionCreateNewScene();
        void actionSaveScene();
        void actionLoadScene();
        void actionLoadFiles();
        void actionSetFocus();
        void actionComputePathAnalysis();
		    void actionLoadCTFiles();
		    void actionSetWaypointFocus();
		    void actionLoadSegmentationsCt();

      private:
        std::string mLastSceneDir;
    };

  }
}
