#include "private/muk.pch"
#include "MukQMenuBar.h"

#include "MukCommon/MukException.h"
#include "MukCommon/geometry.h"

#include <QMenu>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>

#include <qlayout.h>
#include <qlineedit.h>
#include <qlabel.h>
#include <QDialogButtonBox.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <sstream>
#include <windows.h>
#define _USE_MATH_DEFINES
#include <math.h>

namespace
{  
  namespace fs = boost::filesystem;
  void getVersion(const char* libraryName, DWORD& major, DWORD& minor, DWORD& revision, DWORD& build);
}

namespace gris
{
namespace muk
{
  /**
  */
  MukQMenuBar::MukQMenuBar(QWidget* parent)
    : QMenuBar(parent)
    , mFileMenu(nullptr)
    , mOptionsMenu(nullptr)
    , mHelpMenu(nullptr)
    , mLastSceneFiles(nullptr)
  {
    // build the menu bar
    setObjectName("menubar");
    setGeometry(QRect(0, 0, 970, 21));
    
    mFileMenu = new QMenu(this);
    this->addMenu(mFileMenu);
    createFileMenu();    
    mQuickConfigureMenu = new QMenu(this);
    {
      mQuickConfigureMenu->setTitle("Configuration");

      auto* nextAction = new QAction(this);
      nextAction->setText("Open quick planning configuration");
      addAction(mQuickConfigureMenu->menuAction());
      mQuickConfigureMenu->addAction(nextAction);
      connect(nextAction, &QAction::triggered, this, [&] () { emit this->quickPlanningConfigRequested(); });
    }
    mOptionsMenu = new QMenu(this);
    {
      mOptionsMenu->setObjectName("mOptionsMenu"); // useless now, maybe sometime important for qt-translation-mechanism?
      mOptionsMenu->setTitle("Options");

      auto* nextAction = new QAction(this);
      nextAction->setObjectName("actionToggleConsole");
      nextAction->setText("Toggle Console");
      addAction(mOptionsMenu->menuAction());
      mOptionsMenu->addAction(nextAction);
      connect(nextAction, &QAction::triggered, this, &MukQMenuBar::actionShowConsole);

      nextAction = new QAction(this);
      nextAction->setObjectName("actionComputeKappa");
      nextAction->setText("Compute Kappa");
      addAction(mOptionsMenu->menuAction());
      mOptionsMenu->addAction(nextAction);
      connect(nextAction, &QAction::triggered, this, &MukQMenuBar::actionComputeKappa);

      nextAction = new QAction(this);
      nextAction->setObjectName("actionsetDefaultFocus");
      nextAction->setText("Set default focus");
      addAction(mOptionsMenu->menuAction());
      mOptionsMenu->addAction(nextAction);
      connect(nextAction, &QAction::triggered, this, &MukQMenuBar::actionSetDefaultFocus);
    }
    mHelpMenu = new QMenu(this);
    {
      mHelpMenu->setObjectName("mHelpMenu"); // useless now, maybe sometime important for qt-translation-mechanism?
      mHelpMenu->setTitle("Help");

      auto* nextAction = new QAction(this);
      nextAction->setObjectName("actionShowAbout");
      nextAction->setText("About");
      addAction(mHelpMenu->menuAction());
      mHelpMenu->addAction(nextAction);
      connect(nextAction, &QAction::triggered, this, &MukQMenuBar::actionAbout);
    }
    setOrderOfMenus();
  }

  /**
  */
  MukQMenuBar::~MukQMenuBar()
  {
  }

  void MukQMenuBar::setOrderOfMenus()
  {    
    if (nullptr != mFileMenu && nullptr != mOptionsMenu && nullptr != mHelpMenu)
    {
      mHelpMenu->menuAction()->setMenuRole(QAction::AboutRole);
      this->insertMenu(mOptionsMenu->menuAction(), mFileMenu);
      this->insertMenu(mOptionsMenu->menuAction(), mQuickConfigureMenu);
      this->insertMenu(mHelpMenu->menuAction(), mOptionsMenu);
    }
  }

  /** \brief Creates options to load and save MukScenes and (some time ago) MukPaths
  */
  void MukQMenuBar::createFileMenu()
  {    
    mFileMenu->clear();
    mFileMenu->setObjectName("mFileMenu"); // useless now, maybe sometime important for qt-translation-mechanism?
    mFileMenu->setTitle("File");
    
    auto* nextAction = new QAction(this);
    nextAction->setObjectName("actionSavePath");
    nextAction->setText("Save Path (binary)");    
    mFileMenu->addAction(nextAction);
    connect(nextAction, &QAction::triggered, this, &MukQMenuBar::actionSavePath);
      
    nextAction = new QAction(this);
    nextAction->setObjectName("actionSavePath");
    nextAction->setText("Save Path");
    mFileMenu->addAction(nextAction);
    connect(nextAction, &QAction::triggered, this, &MukQMenuBar::actionSavePath);

    nextAction = new QAction(this);
    nextAction->setObjectName("actionLoadPath");
    nextAction->setText("Load Path");
    mFileMenu->addAction(nextAction);
    connect(nextAction, &QAction::triggered, this, &MukQMenuBar::actionLoadPath);

    auto menuLoadAs = new QMenu("Load As", this);
    {
      nextAction = new QAction(this);
      nextAction->setObjectName("actionLoadAsObstacle");
      nextAction->setText("Obstacle");
      menuLoadAs->addAction(nextAction);
      connect(nextAction, &QAction::triggered, this, &MukQMenuBar::actionLoadAsObstacle);
      mFileMenu->addMenu(menuLoadAs);
    }

    mFileMenu->addSeparator();
    if (nullptr != mLastSceneFiles)
    {
      for (const auto& file : *mLastSceneFiles)
      {
        nextAction = new QAction(mFileMenu);
        nextAction->setText(file.c_str());
        mFileMenu->addAction(nextAction);
        connect(nextAction, &QAction::triggered, [this, nextAction] () 
          {
            emit this->loadSceneClicked(nextAction->text().toLocal8Bit().constData()); 
          });
      }
    }

    setOrderOfMenus();
  }

  /**
  */
  void MukQMenuBar::setLastSceneFiles(const std::list<std::string>& v)
  {
    mLastSceneFiles = &v;
    createFileMenu();
    mLastSceneFiles = nullptr;
  }

  /**
  */
  void MukQMenuBar::actionShowConsole()
  {
    if (IsWindowVisible(GetConsoleWindow()))
    {
      ShowWindow( GetConsoleWindow(), SW_HIDE );
      LOG_LINE << "Changed Console Visibility to OFF";
    }
    else
    {
      ShowWindow( GetConsoleWindow(), SW_RESTORE );
      LOG_LINE << "Changed Console Visibility to ON";
    }    
  }

  /**
  */
  void MukQMenuBar::actionSetDefaultFocus()
  {
    emit setDefaultFocusClicked();
  }

  /**
  */
  void MukQMenuBar::actionComputeKappa()
  {
    auto pDialog = std::make_unique<MultipleInputDialog>();
    pDialog->exec();
  }

  /**
  */
  void MukQMenuBar::actionSavePath()
  {
    QString qFilename = QFileDialog::getSaveFileName(this, tr("Save as"), "./",  tr("Text File (*.txt)"));
    if (qFilename.isEmpty())
      return;
    emit savePathClicked(qFilename.toLocal8Bit().constData());    
  }

  /**
  */
  void MukQMenuBar::actionLoadPath()
  {
    QString qFilename = QFileDialog::getOpenFileName(this, tr("Load Path"), "./", tr("Text File (*.txt)"));
    if (qFilename.isEmpty())
      return;
    emit loadPathClicked(qFilename.toLocal8Bit().constData());
  }
  
  /**
  */
  void MukQMenuBar::actionAbout()
  {    
    auto pBox = std::make_unique<QMessageBox>();
    pBox->setWindowTitle("About Mukno Planning Tool");
    pBox->setStyleSheet("QMessageBox { background-color: lightGray; }");
    std::stringstream ss;
    {
      DWORD major, minor, revision, build;
      ::getVersion("MuknoPlanner.exe", major, minor, revision, build);
      ss << "Mukno Planning Tool\n";
      ss << (boost::format("Version: %d.%d.%d.%d\n") % major % minor % revision % build).str() ;
      ss<< "\n";

      ss << "Developed at\n";
      ss << "   Technische Universitaet Darmstadt\n";
      ss << "   Interactive Graphics Systems Group\n";
      ss << "   http://www.gris.tu-darmstadt.de/home/index.en.htm\n";
      ss << "\n";

      ss << "For details about this project see\n";
      ss << "   http://www.mukno.de/index.en.html\n";
      ss << "Financed by the German Research Foundation\n";
      ss << "   http://www.dfg.de/en\n";
      ss << "\n";

      ss << "Contributors:\n";
      ss << "   Johannes Fauser\n";
      ss << "   David Kuegler\n";
      ss << "   Andreas Franke\n";
      ss << "   Georgios Sakas\n";
      ss << "   Julia Kristin\n";
      ss << "   Igor Stenin\n";
      ss << "\n";

      ss << "Contact \n";
      ss << "   johannes.fauser@gris.tu-darmstadt.de\n";
      ss << "\n";

      ss << "This application makes use of\n";
      ss << "   boost 1.59\n" ;
      ss << "   CGAL 4.7\n" ;
      ss << "   Eigen 3.2.7\n" ;
      ss << "   ITK 4.10.1\n" ;
      ss << "   loki-0.1.7\n" ;
      ss << "   OMPL 1.2\n" ;
      ss << "   PlusToolkit-2.5\n" ;
      ss << "   qcustomplot 1.3.2\n" ;
      ss << "   Qt 5.5\n" ;
      ss << "   VTK 6.3\n" ;
      
    }    
    pBox->setText(ss.str().c_str());
    pBox->exec();
  }

  /**
  */
  void MukQMenuBar::actionLoadAsObstacle()
  {
    QString qFilename = QFileDialog::getOpenFileName(this, tr("Obstacle"), "./", tr("Files (*.vtk, *.mhd, *.stl)"));
    if (qFilename.isEmpty())
      return;
    emit loadAsObstacle(qFilename.toLocal8Bit().constData());
  }

  // ==================================================================================================================================

  /**
  */
  MultipleInputDialog::MultipleInputDialog()
  {
    auto* vbox = new QGridLayout(this);
    mLineEditAngle  = new QLineEdit(this);
    mLineEditLength = new QLineEdit(this);
    mLineEditKappa  = new QLineEdit(this);

    auto* labelAngle  = new QLabel("Angle in degrees", this);
    auto* labelLength = new QLabel("Length in millimeters", this);
    auto* labelKappa  = new QLabel("Kappa in 1/mm", this);
        
    vbox->addWidget(labelAngle, 0, 0);
    vbox->addWidget(labelLength, 1, 0);
    vbox->addWidget(labelKappa, 2, 0);

    vbox->addWidget(mLineEditAngle, 0, 1);
    vbox->addWidget(mLineEditLength, 1, 1);
    vbox->addWidget(mLineEditKappa, 2, 1);;
    
    mButtonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    vbox->addWidget(mButtonBox);

    connect(mButtonBox, &QDialogButtonBox::accepted, this, &MultipleInputDialog::computeAndShowKappa);
    connect(mButtonBox, &QDialogButtonBox::rejected, this, &MultipleInputDialog::reject);

    this->setLayout(vbox);
  }

  /**
  */
  void MultipleInputDialog::computeAndShowKappa()
  {
    auto strAngle  = mLineEditAngle->text().toLocal8Bit().constData();
    auto strLength = mLineEditLength->text().toLocal8Bit().constData();
    const double angle  ( static_cast<double>( std::stof(strAngle) ) * M_PI / 180);
    const double length ( static_cast<double>( std::stof(strLength)) );
    double kappa(0);
    maxCurvatureFromMaxAngle(angle, length, kappa);
    mLineEditKappa->setText(QString::number(kappa));
  }
}
}


namespace
{
  void getVersion(const char* libraryName, DWORD& major, DWORD& minor, DWORD& revision, DWORD& build)
  {
    using namespace gris::muk;
    DWORD               dwSize              = 0;
    BYTE                *pbVersionInfo      = NULL;
    VS_FIXEDFILEINFO    *pFileInfo          = NULL;
    UINT                puLenFileInfo       = 0;

    // get the version info for the file requested
    dwSize = GetFileVersionInfoSize( libraryName, NULL );
    BOOL valid = dwSize != 0;
    if (valid)
    {
      pbVersionInfo = new BYTE[ dwSize ];
      if (valid = GetFileVersionInfo( libraryName, 0, dwSize, pbVersionInfo))
      {
        if (valid = GetFileVersionInfo(libraryName, 0, dwSize, pbVersionInfo))
        {
          if (valid = VerQueryValue(pbVersionInfo, TEXT("\\"), (LPVOID*)&pFileInfo, &puLenFileInfo))
          {
            major     = (pFileInfo->dwFileVersionMS >> 16) & 0xffff;
            minor     = (pFileInfo->dwFileVersionMS >>  0) & 0xffff;
            revision  = (pFileInfo->dwFileVersionLS >> 16) & 0xffff;
            build     = (pFileInfo->dwFileVersionLS >>  0) & 0xffff;
          }
        }        
      }
      delete[] pbVersionInfo;      
    }
    if (!valid)
    {
      throw MUK_EXCEPTION((boost::format("Checking Library %s failed!") % libraryName).str().c_str(), (boost::format("Error Code: %d") % GetLastError()).str().c_str());
    }
  }
}