#include "private/muk.pch"
#include "MukQToolBar.h"

#include "MukCommon/MukException.h"

#include <QAction>
#include <QFileDialog>
#include <qsignalmapper.h>

#include <functional>

namespace
{
  enum EnActions
  {
    enNewScene = 0,
    enSaveScene,
    enLoadScene,
    enLoadFiles,
    enSetFocus,
    enPathAnalysis,
    enLoadCTFiles,
    enSetWaypointFocus,
    enLoadSegmentationsCt,
    Size_EnActions
  };
}

namespace gris
{
  namespace muk
  {
    /**
    */
    MukQToolBar::MukQToolBar(QWidget* parent)
      : QToolBar(parent)
    {
      setObjectName(QStringLiteral("toolbar"));
      setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

      for (int i(0); i<Size_EnActions; ++i)
      {
        addAction(new QAction(this));
      }

      auto* nextAction = actions()[enNewScene];
      {        
        nextAction->setObjectName("actionNewScene");
        nextAction->setText("Clear Scene");
        nextAction->setToolTip("Remove everything");
        nextAction->setShortcut(QKeySequence("Strg+N"));
        QIcon icon;
        icon.addFile("../resources/icons/windows/Delete.png", QSize(), QIcon::Normal, QIcon::Off);
        nextAction->setIcon(icon);
      }
      nextAction = actions()[enSaveScene];
      {
        nextAction->setObjectName("actionSaveScene");
        nextAction->setText("Save Scene");
        nextAction->setToolTip("Saves configurations, paths, paths to old obstacles etc.");
        nextAction->setShortcut(QKeySequence("Strg+S"));
        QIcon icon;
        icon.addFile("../resources/icons/windows/Save.png", QSize(), QIcon::Normal, QIcon::Off);
        nextAction->setIcon(icon);
      }
      nextAction = actions()[enLoadScene];
      {
        nextAction->setObjectName("actionLoadScene");
        nextAction->setText("Load Scene");
        nextAction->setToolTip("Read a scene from a text file");
        nextAction->setShortcut(QKeySequence("Strg+L"));
        QIcon icon;
        icon.addFile("../resources/icons/windows/Folder.png", QSize(), QIcon::Normal, QIcon::Off);
        nextAction->setIcon(icon);
      }
      nextAction = actions()[enLoadFiles];
      {
        nextAction->setObjectName("actionLoadFiles");
        nextAction->setText("Load Files");
        nextAction->setToolTip("add segmented CT-Data to the scene");
        nextAction->setShortcut(QKeySequence("Strg+L"));
        QIcon icon;
        icon.addFile("../resources/icons/windows/Folder.png", QSize(), QIcon::Normal, QIcon::Off);
        nextAction->setIcon(icon);
      }
      nextAction = actions()[enSetFocus];
      {
        nextAction->setObjectName("actionSetFocus");
        nextAction->setText("Default Focus");
        nextAction->setToolTip("Set camera focus in the middle of the structures");
        QIcon icon;
        icon.addFile("../resources/icons/windows/View.png", QSize(), QIcon::Normal, QIcon::Off);
        nextAction->setIcon(icon);
      }
      nextAction = actions()[enPathAnalysis];
      {
        nextAction->setObjectName("actionPathAnalysis");
        nextAction->setText("Path Analysis");
        nextAction->setToolTip("Compute and plot the distance and curvature at each point of the current path");
        QIcon icon;
        icon.addFile("../resources/icons/windows/Info.png", QSize(), QIcon::Normal, QIcon::Off);
        nextAction->setIcon(icon);
      }      
	  nextAction = actions()[enLoadCTFiles];
	  {
		  nextAction->setObjectName("actionLoadCTFiles");
		  nextAction->setText("Load CT Files");
		  nextAction->setToolTip("Loads the CT-Data into the CT window");
		  QIcon icon;
		  icon.addFile("../resources/icons/windows/Folder.png", QSize(), QIcon::Normal, QIcon::Off);
		  nextAction->setIcon(icon);
	  }
	  nextAction = actions()[enSetWaypointFocus];
	  {
		  nextAction->setObjectName("actionSetWaypointFocus");
		  nextAction->setText("Focus Waypoint");
		  nextAction->setToolTip("Set camera focus in CT window to 1st waypoint");
		  QIcon icon;
		  icon.addFile("../resources/icons/windows/View.png", QSize(), QIcon::Normal, QIcon::Off);
		  nextAction->setIcon(icon);
	  }
	  nextAction = actions()[enLoadSegmentationsCt];
	  nextAction->setObjectName("actionLoadSegmentationsCt");
	  nextAction->setText("Load Segmentation");
	  nextAction->setToolTip("Breaks down a single segmentation file into multiple segmentations and loads these into the ct viewer");
	  QIcon icon;
	  icon.addFile("../resources/icons/windows/Folder.png", QSize(), QIcon::Normal, QIcon::Off);
	  nextAction->setIcon(icon);

    }

    /**
    */
    MukQToolBar::~MukQToolBar()
    {
    }

    /**
    */
    void MukQToolBar::setupConnections()
    {
      connect(actions()[enNewScene],  &QAction::triggered, this, &MukQToolBar::actionCreateNewScene);
      connect(actions()[enSaveScene], &QAction::triggered, this, &MukQToolBar::actionSaveScene);
      connect(actions()[enLoadScene], &QAction::triggered, this, &MukQToolBar::actionLoadScene);
      connect(actions()[enLoadFiles], &QAction::triggered, this, &MukQToolBar::actionLoadFiles);
      connect(actions()[enSetFocus],  &QAction::triggered, this, &MukQToolBar::actionSetFocus);
      connect(actions()[enPathAnalysis], &QAction::triggered, this, &MukQToolBar::actionComputePathAnalysis);
	    connect(actions()[enLoadCTFiles], &QAction::triggered, this, &MukQToolBar::actionLoadCTFiles);
	    connect(actions()[enSetWaypointFocus], &QAction::triggered, this, &MukQToolBar::actionSetWaypointFocus);
	    connect(actions()[enLoadSegmentationsCt], &QAction::triggered, this, &MukQToolBar::actionLoadSegmentationsCt);
    }
    
    /**
    */
    void MukQToolBar::actionCreateNewScene()
    {
      emit createNewSceneClicked();
    }

    /**
    */
    void MukQToolBar::actionSaveScene()
    {
      std::stringstream ss;
      ss << mLastSceneDir << "/" << "scene.mukscene";
      QString qFilename = QFileDialog::getSaveFileName(this, tr("Save as"),
        ss.str().c_str(), tr("XML Files (*.xml *.mukscene)"));
      if (qFilename.isEmpty())
        return;
      emit saveSceneClicked(qFilename.toLocal8Bit().constData());
    }

    /**
    */
    void MukQToolBar::actionLoadScene()
    {
      QString qFilename = QFileDialog::getOpenFileName(this, tr("Load Scene"), mLastSceneDir.c_str(), tr("XML File (*.xml *.mukscene)"));
      if (qFilename.isEmpty())
        return;
      emit loadSceneClicked(qFilename.toLocal8Bit().constData());
    }

    /** \brief Lets the user select multiplee files and sends a signal for each of them
    */
    void MukQToolBar::actionLoadFiles()
    {
      QStringList fileNames = QFileDialog::getOpenFileNames(this,
        tr("Open Files"), "", tr("Image Files (*.mhd *.vtk *.obj *.stl)"));
      if (fileNames.isEmpty())
        return;
            
      for (int i(0); i<fileNames.size(); ++i)
      {
        std::string str(fileNames[i].toLocal8Bit().constData());
        emit loadObstacleFileClicked(str);
      }      
    }

    /**
    */
    void MukQToolBar::actionSetFocus()
    {
      emit focusClicked();
    }

    /**
    */
    void MukQToolBar::actionComputePathAnalysis()
    {
      emit pathAnalysisRequested();
    }

	/**
	*/
	void MukQToolBar::actionLoadCTFiles()
	{
		QString qFilename = QFileDialog::getOpenFileName(this, tr("Open CT File"), "", tr("TImage Files (*.mhd *.vtk *.obj)"));
		if (qFilename.isEmpty())
			return;
		emit loadCTFileClicked(qFilename.toLocal8Bit().constData());		
	}

	/**
	*/
	void MukQToolBar::actionSetWaypointFocus()
	{
		emit focusOnWaypointClicked();
	}

	/**
	*/
	void MukQToolBar::actionLoadSegmentationsCt()
	{
		QString qFilename = QFileDialog::getOpenFileName(this, tr("Open Segmentations File"), "", tr("TImage Files (*.mhd *.raw *.nrrd)"));
		if (qFilename.isEmpty())
			return;
		emit loadSegmentationFileClicked(qFilename.toLocal8Bit().constData());
	}

  }
}
