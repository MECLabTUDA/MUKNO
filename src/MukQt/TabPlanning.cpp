#include "private/muk.pch"
#include "TabPlanning.h"

#include "muk_qt_tools.h"

#include "MukCommon/MukException.h"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSplitter>
#include <qscrollarea.h>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <qmainwindow.h>
#include <QGridLayout>
#include <qlabel.h>
#include <qcombobox.h>
#include <qpushbutton.h>
#include <QLineEdit>
#include <qspinbox.h>
#include <qapplication.h>
#include <qinputdialog.h>
#include <qscrollarea.h>
#include <qradiobutton.h>
#include <qgroupbox.h>
#include <qbuttongroup.h>
#include <QVTKWidget.h>

#include <boost/format.hpp>

namespace
{
  const int Qt_Window_Precision = 2;
}

namespace gris
{
namespace muk
{
  /**
  */
  TabPlanning::TabPlanning(QWidget *parent)
    : QWidget(parent)
  {
    // The tab has a obligatory layout that manages the scroll area
    auto* mainLayout     = new QVBoxLayout(this);
    auto* scrollArea     = new QScrollArea(this);
    mainLayout->addWidget(scrollArea);
    // The scroll area holds a single widget "tabContents"
    auto* tabContents    = new QWidget(scrollArea);  // default policy : ScrollBarAsNeeded
    auto* contentLayout = new QVBoxLayout(tabContents);
    scrollArea->setWidget(tabContents);
    scrollArea->setFrameShape(QFrame::NoFrame); // makes sure the tabContent-Widget takes space of the whole scroll area
    mainLayout->setContentsMargins(0, 0, 0, 0); // makes sure the tabContent-Widget takes space of the whole parent widget
    scrollArea->setBackgroundRole(QPalette::ColorRole::Light);
    // The layout of the contents
    contentLayout->setSizeConstraint(QLayout::SetMinimumSize);
    const auto rootResourceFiles = std::string("../resources/icons/windows/");
    const auto resourceAdd     = "Add.png";
    const auto resourceDelete  = "Delete.png";
    const auto resourceRefresh = "Refresh.png";

    // layout PathCollection + MukPaths
    auto* layout = new QGridLayout();
    {
      QIcon icon;
      int row(0);
      mpPathSelection = new QComboBox(tabContents);
      // combobox activePath
      auto* labelPathCollection = new QLabel(tabContents);
      { 
        labelPathCollection->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
        labelPathCollection->setText("Path Collection:");
        layout->addWidget(labelPathCollection, row, 0, 1, 1);

        mpPathSelection->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
        connect(mpPathSelection, &QComboBox::currentTextChanged, [&] (const QString& str) { emit pathCollectionChanged(str.toLocal8Bit().constData()); });
        layout->addWidget(mpPathSelection, row, 1, 1, 2);
        ++row;
      }
      auto* buttonAddPathCollection = new QToolButton(tabContents);
      {
        buttonAddPathCollection->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
        buttonAddPathCollection->setText("Add Path Collection");
        const auto resFile = rootResourceFiles + resourceAdd;
        icon.addFile(resFile.c_str(), QSize(), QIcon::Normal, QIcon::Off);
        buttonAddPathCollection->setIcon(icon);
        buttonAddPathCollection->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        connect(buttonAddPathCollection, &QToolButton::clicked, [=] (bool) { this->addPathCollection(mpPathSelection); });
        layout->addWidget(buttonAddPathCollection, row, 1, 1, 1);
      }
      auto* buttonDeletePathCollection = new QToolButton(tabContents);
      {
        buttonDeletePathCollection->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
        buttonDeletePathCollection->setText("Delete Path Collection");
        const auto resFile = rootResourceFiles + resourceDelete;
        icon.addFile(resFile.c_str(), QSize(), QIcon::Normal, QIcon::Off);
        buttonDeletePathCollection->setIcon(icon);
        buttonDeletePathCollection->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        connect(buttonDeletePathCollection, &QToolButton::clicked, this, SELECT<bool>::OVERLOAD_OF(&TabPlanning::deletePathCollection));
        layout->addWidget(buttonDeletePathCollection, row, 2, 1, 1);
        ++row;
      }
      auto* buttonClearPathCollection = new QToolButton(tabContents);
      {
        buttonClearPathCollection->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
        buttonClearPathCollection->setText("Clear Path Collection");
        const auto resFile = rootResourceFiles + resourceDelete;
        icon.addFile(resFile.c_str(), QSize(), QIcon::Normal, QIcon::Off);
        buttonClearPathCollection->setIcon(icon);
        buttonClearPathCollection->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        connect(buttonClearPathCollection, &QToolButton::clicked, this, SELECT<bool>::OVERLOAD_OF(&TabPlanning::clearPathCollection));
        layout->addWidget(buttonClearPathCollection, row, 2, 1, 1);
        ++row;
      }
      // add, change MukPath
      {
        auto* labelCreateMukPaths = new QLabel(tabContents);
        labelCreateMukPaths->setText("Create Paths");
        labelCreateMukPaths->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
        layout->addWidget(labelCreateMukPaths, row, 0, 1, 1);
        {
          auto* spinBox = new QSpinBox(tabContents);
          spinBox->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
          spinBox->setMinimum(1);
          layout->addWidget(spinBox, row, 1, 1, 1);
          auto* button = new QToolButton(tabContents);
          button->setText("Create (#)");
          button->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
          auto resFile = rootResourceFiles + resourceRefresh;
          icon.addFile(resFile.c_str(), QSize(), QIcon::Normal, QIcon::Off);
          button->setIcon(icon);
          button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
          connect(button, &QToolButton::clicked, [=] (bool) { this->createMukPathsNumber(mpPathSelection, spinBox); });
          layout->addWidget(button, row, 2, 1, 1);
          ++row;

          auto* spinBoxTime = new QDoubleSpinBox(tabContents);
          spinBoxTime->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
          spinBoxTime->setMinimum(0.0);
          spinBoxTime->setValue(1.0);
          layout->addWidget(spinBoxTime, row, 1, 1, 1);
          button = new QToolButton(tabContents);
          button->setText("Create (s)");
          button->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
          resFile = rootResourceFiles + resourceRefresh;
          icon.addFile(resFile.c_str(), QSize(), QIcon::Normal, QIcon::Off);
          button->setIcon(icon);
          button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
          connect(button, &QToolButton::clicked, [=] (bool) { this->createMukPathsTime(mpPathSelection, spinBoxTime); });
          layout->addWidget(button, row, 2, 1, 1);
          ++row;
        }

        auto* labelUpdateMukPath = new QLabel(tabContents);
        {
          labelUpdateMukPath->setText("Update Path");
          labelUpdateMukPath->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
          layout->addWidget(labelUpdateMukPath, row, 0, 1, 1);
        }
        mpPathIdxSpinBox= new QSpinBox(tabContents);
        {
          mpPathIdxSpinBox->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
          mpPathIdxSpinBox->setMinimum(-1);
          connect(mpPathIdxSpinBox, SELECT<int>::OVERLOAD_OF(&QSpinBox::valueChanged), [this] (int idx) { emit this->pathIdxChanged(idx); });
          layout->addWidget(mpPathIdxSpinBox, row, 1, 1, 1);
        }
        auto* buttonUpdateMukPaths = new QToolButton(tabContents);
        {
          buttonUpdateMukPaths->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
          buttonUpdateMukPaths->setText("Update");
          const auto resFile = rootResourceFiles + resourceRefresh;
          icon.addFile(resFile.c_str(), QSize(), QIcon::Normal, QIcon::Off);
          buttonUpdateMukPaths->setIcon(icon);
          buttonUpdateMukPaths->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
          connect(buttonUpdateMukPaths, &QToolButton::clicked, this, SELECT<bool>::OVERLOAD_OF(&TabPlanning::updateMukPath));
          layout->addWidget(buttonUpdateMukPaths, row, 2, 1, 1);
          ++row;
        }
        mpPushButtonCopy = new QPushButton(tabContents);
        {
          mpPushButtonCopy->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
          mpPushButtonCopy->setText("Copy");
          connect(mpPushButtonCopy, &QToolButton::clicked, this, &TabPlanning::copyMukPath);
          layout->addWidget(mpPushButtonCopy, row, 2, 1, 1);
          ++row;
        }
        auto* nextLabel = new QLabel(tabContents);
        {
          nextLabel->setText("State");
          nextLabel->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
          layout->addWidget(nextLabel, row, 0, 1, 1);
        }
        mpStateIdxSpinBox = new QSpinBox(tabContents);
        {
          mpStateIdxSpinBox->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
          mpStateIdxSpinBox->setMinimum(0);
          layout->addWidget(mpStateIdxSpinBox, row, 1, 1, 1);
        }
        auto* nextToolButton = new QToolButton(tabContents);
        {
          nextToolButton->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
          nextToolButton->setText("Add Replanning");
          const auto resFile = rootResourceFiles + resourceAdd;
          icon.addFile(resFile.c_str(), QSize(), QIcon::Normal, QIcon::Off);
          nextToolButton->setIcon(icon);
          nextToolButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
          connect(nextToolButton, &QToolButton::clicked, this, [&] ()
            {
              const auto str      = mpPathSelection->currentText().toStdString();
              const auto pathIdx  = mpPathIdxSpinBox->value();
              const auto stateIdx = mpStateIdxSpinBox->value();
              emit addReplanningClicked(str, pathIdx, stateIdx);
            });
          layout->addWidget(nextToolButton, row, 2, 1, 1);
          ++row;
        }
      }
      contentLayout->addLayout(layout);
    }
    
    // we do not care about a horizontal but a enforced vertical spacing
    // -> 0 dummy width but a minimum 40 pixel height, enforced by the minimumexpanding property
    auto spacer = new QSpacerItem(0, 40, QSizePolicy::Minimum, QSizePolicy::MinimumExpanding); 
    contentLayout->addItem(spacer);

    // layout PathPlanner + Pruner + Interpolator
    auto* gridLayoutPlanning = new QGridLayout();
    {
      int currentRow = 0;
      auto* labelPlanner = new QLabel(tabContents);
      comboBoxPlanner    = new QComboBox(tabContents);
      pushButtonPlanner  = new QPushButton(tabContents);
      
      /*mpSamplingBox = new QGroupBox(tabContents);
      {
        mpSampling1 = new QRadioButton(tabContents);
        mpSampling2 = new QRadioButton(tabContents);
        mpSamplingGroup = new QButtonGroup(tabContents);
        auto* groupLayout = new QHBoxLayout();
        groupLayout->addWidget(mpSampling1);
        groupLayout->addWidget(mpSampling2);
        mpSamplingGroup ->addButton(mpSampling1);
        mpSamplingGroup ->addButton(mpSampling2);
        mpSamplingBox->setLayout(groupLayout);
        mpSampling1->setChecked(true);
      }*/
      {
        labelPlanner->setText("Planner");
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(comboBoxPlanner->sizePolicy().hasHeightForWidth());
        comboBoxPlanner->setSizePolicy(sizePolicy);
        connect(comboBoxPlanner, &QComboBox::currentTextChanged, [&] (const QString& str) { emit plannerChanged(str.toLocal8Bit().constData()); } );
        pushButtonPlanner->setText("Update");
        connect(pushButtonPlanner, &QPushButton::clicked, this, &TabPlanning::updatePlannerClicked);

        /*gridLayoutPlanning->addWidget(mpSamplingBox, currentRow, 2, 1, 1);
        connect(mpSamplingGroup, SELECT<int>::OVERLOAD_OF(&QButtonGroup::buttonClicked), [&] (int i) { emit this->changeSamplingClicked(); });
        ++currentRow;*/

        gridLayoutPlanning->addWidget(labelPlanner, currentRow, 0, 1, 1);
        gridLayoutPlanning->addWidget(comboBoxPlanner, currentRow, 1, 1, 1);
        gridLayoutPlanning->addWidget(pushButtonPlanner, currentRow, 2, 1, 1);
        ++currentRow;
      }
      auto* labelPruner = new QLabel(tabContents);
      comboBoxPruner = new QComboBox(tabContents);
      pushButtonPruner = new QPushButton(tabContents);
      {
        labelPruner->setMouseTracking(false);
        labelPruner->setText("Pruner");
        connect(comboBoxPruner, &QComboBox::currentTextChanged, [&] (const QString& str) { emit prunerChanged(str.toLocal8Bit().constData()); } );
        pushButtonPruner->setText("Update");
        connect(pushButtonPruner, &QPushButton::clicked, this, &TabPlanning::updatePrunerClicked);
        gridLayoutPlanning->addWidget(labelPruner, currentRow, 0, 1, 1);
        gridLayoutPlanning->addWidget(comboBoxPruner, currentRow, 1, 1, 1);
        gridLayoutPlanning->addWidget(pushButtonPruner, currentRow, 2, 1, 1);
        ++currentRow;
      }
      auto* labelInterpolation = new QLabel(tabContents);
      comboBoxInterpolator = new QComboBox(tabContents);
      pushButtonInterpolator = new QPushButton(tabContents);
      {
        labelInterpolation->setMouseTracking(false);
        labelInterpolation->setText("Interpolation");
        connect(comboBoxInterpolator, &QComboBox::currentTextChanged, [&] (const QString& str) { emit interpolatorChanged(str.toLocal8Bit().constData()); } );
        pushButtonInterpolator->setText("Update");
        connect(pushButtonInterpolator, &QPushButton::clicked, this, &TabPlanning::updateInterpolatorClicked);
        gridLayoutPlanning->addWidget(labelInterpolation, currentRow, 0, 1, 1);
        gridLayoutPlanning->addWidget(comboBoxInterpolator, currentRow, 1, 1, 1);
        gridLayoutPlanning->addWidget(pushButtonInterpolator, currentRow, 2, 1, 1);
        ++currentRow;
      }
      mpPushButtonUpdateAll = new QPushButton(tabContents);
      {
        mpPushButtonUpdateAll->setText("Update");
        connect(mpPushButtonUpdateAll, &QPushButton::clicked, [&] (void) { emit updatePlannerClicked(); emit updatePrunerClicked(); emit updateInterpolatorClicked(); } );
        gridLayoutPlanning->addWidget(mpPushButtonUpdateAll, currentRow, 2, 1, 1);
        ++currentRow;
      }
      contentLayout->addLayout(gridLayoutPlanning);
    }

    // we do not care about a horizontal but a enforced vertical spacing
    // -> 0 dummy width but a minimum 40 pixel height, enforced by the minimumexpanding property
    spacer = new QSpacerItem(0, 40, QSizePolicy::Minimum, QSizePolicy::MinimumExpanding); 
    contentLayout->addItem(spacer);

    auto * toolButtonUpdateProblemDefinition = new QToolButton(tabContents);
    {
      toolButtonUpdateProblemDefinition->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
      toolButtonUpdateProblemDefinition->setText("Update ProblemDefinition");
      QIcon icon;
      const auto resFile = rootResourceFiles + resourceRefresh;
      icon.addFile(resFile.c_str(), QSize(), QIcon::Normal, QIcon::Off);
      toolButtonUpdateProblemDefinition->setIcon(icon);
      toolButtonUpdateProblemDefinition->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
      connect(toolButtonUpdateProblemDefinition, &QToolButton::clicked, this, &TabPlanning::updateProblemDefinition);
      contentLayout->addWidget(toolButtonUpdateProblemDefinition);
    }

    // layout Waypoint Visualization + Selection
    mpProbDefWidget = new ProblemDefinitionWidget(tabContents);
    contentLayout->addWidget(mpProbDefWidget);
  }

  /**
  */
  void TabPlanning::setPathCollection(const std::string& name)
  {
    QSignalBlocker b(mpPathSelection);
    auto index = mpPathSelection->findText(name.c_str());
    if (index<0)
      return;
    mpPathSelection->setCurrentIndex(index);
  }

  /** \brief sets the value of the spinbox mpPathIdxSpinBox
  */
  void TabPlanning::setActivePathIdx(int value)
  {
    const QSignalBlocker blocker(mpPathIdxSpinBox);
    mpPathIdxSpinBox->setValue(value);
  }

  /** \brief sets the value of the spinbox mpStateIdxSpinBox
  */
  void TabPlanning::setStateIdx(int value)
  {
    const QSignalBlocker blocker(mpStateIdxSpinBox);
    mpStateIdxSpinBox->setValue(value);
  }


  /** \brief evaluates click on button "addPathCollection"
  */
  void TabPlanning::addPathCollection(QComboBox* box)
  {
    auto str = (boost::format("PathCollection_%02d") % box->count()).str();
    bool ok;
    QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),
      tr("PathCollection name:"), QLineEdit::Normal, str.c_str(), &ok);    
    if (ok && !text.isEmpty())
    {
      emit addPathCollectionClicked(text.toLocal8Bit().constData());
    }
  }

  /** \brief evaluates selection of combobox "comboBoxPathSelection"
  */
  void TabPlanning::deletePathCollection(bool)
  {
    if (mpPathSelection->currentIndex() != -1)
    {
      emit deletePathCollectionClicked(mpPathSelection->currentText().toLocal8Bit().constData());
      mpPathSelection->removeItem(mpPathSelection->currentIndex());
    }    
  }

  /** \brief evaluates click on button "clearPathCollection"
  */
  void TabPlanning::clearPathCollection(bool)
  {
    if (mpPathSelection->currentIndex() != -1)
    {
      std::string name = mpPathSelection->currentText().toLocal8Bit().constData();
      emit clearPathCollectionClicked(name);
    }    
  }

  /**
  */
  void TabPlanning::createMukPathsNumber(QComboBox* box, QSpinBox* spin)
  {
    if (box->currentIndex() != -1)
    {
      std::string name = box->currentText().toLocal8Bit().constData();
      const size_t n   = spin->value();
      emit createMukPathsClicked(name, n);
    }
  }

  /**
  */
  void TabPlanning::createMukPathsTime(QComboBox* box, QDoubleSpinBox* spin)
  {
    if (box->currentIndex() != -1)
    {
      std::string name = box->currentText().toLocal8Bit().constData();
      const auto d   = spin->value();
      emit createMukPathsClicked(name, d);
    }
  }

  /**
  */
  void TabPlanning::updateMukPath(bool)
  {
    std::string name = mpPathSelection->currentText().toLocal8Bit().constData();
    const int idx   = mpPathIdxSpinBox->value()-1;
    emit updateMukPathClicked(name, idx); 
  }

  /**
  */
  void TabPlanning::copyMukPath()
  {
    std::string name = mpPathSelection->currentText().toLocal8Bit().constData();
    const int idx    = mpPathIdxSpinBox->value();
    emit copyMukPathClicked(name, idx); 
  }

  /**
  */
  void TabPlanning::updateProblemDefinition(bool)
  {
    if (mpPathSelection->currentIndex() != -1)
    {
      std::string name = mpPathSelection->currentText().toLocal8Bit().constData();
      emit updateProblemDefinitionClicked(name);
    }
  }

  /**
  */
  void TabPlanning::insertPathCollection(const std::string& key)
  {
    mpPathSelection->addItem(key.c_str());
    mpPathSelection->setCurrentIndex(mpPathSelection->count()-1);
  }

  /**
  */
  int TabPlanning::getSamlingMethod() const
  {
   /* if (mpSampling1->isChecked())
      return 1;
    else 
      return 2;*/
    return 1;
  }

  /**
  */
  void TabPlanning::setSamplingText(const std::string& str1, const std::string& str2)
  {
    /*QSignalBlocker blocker(mpSamplingBox);
    mpSampling1->setText(str1.c_str());
    mpSampling2->setText(str2.c_str());*/
  }

  /**
  */
  void TabPlanning::setSamplingSelection(int i)
  {
    /*QSignalBlocker blocker(mpSamplingBox);
    if (i==1)
      mpSampling1->setChecked(true);
    else if (i==2)
      mpSampling2->setChecked(true);
    else
    {
      mpSampling2->setChecked(false);
      mpSampling2->setChecked(false);
    }*/
  }

  /**
  */
  void TabPlanning::retranslateUi(QMainWindow *MainWindow)
  {
  }

  /**
  */
  TabPlanning::~TabPlanning()
  {
  }

  /**
  */
  void TabPlanning::setPlanners(const std::vector<std::string>& keys)
  {
    QSignalBlocker(this);
    comboBoxPlanner->clear();
    std::for_each(keys.begin(), keys.end(), [&] (const std::string& key) { comboBoxPlanner->addItem(key.c_str()); });
  }

  /**
  */
  void TabPlanning::setPruners(const std::vector<std::string>& keys)
  {
    QSignalBlocker(this);
    comboBoxPruner->clear();
    std::for_each(keys.begin(), keys.end(), [&] (const std::string& key) { comboBoxPruner->addItem(key.c_str()); });
  }

  /**
  */
  void TabPlanning::setInterpolators(const std::vector<std::string>& keys)
  {
    QSignalBlocker(this);
    comboBoxInterpolator->clear();
    std::for_each(keys.begin(), keys.end(), [&] (const std::string& key) { comboBoxInterpolator->addItem(key.c_str()); });
  }

  /**
  */
  void TabPlanning::setPathCollections(const std::vector<std::string>& keys, int current)
  {
    QSignalBlocker blocker(this);
    mpPathSelection->clear();
    std::for_each(keys.begin(), keys.end(), [&] (const std::string& key) { mpPathSelection->addItem(key.c_str()); });
    if (current >= 0 && current < keys.size())
    {
      mpPathSelection->setCurrentIndex(current);
    }
  }

  /**
  */
  void TabPlanning::setPlanner(const std::string& key)
  {
    auto index = comboBoxPlanner->findText(key.c_str());
    if (index<0)
      LOG_LINE << "Could not find planner";
    else
      comboBoxPlanner->setCurrentIndex(index);
  }

  /**
  */
  void TabPlanning::setPruner(const std::string& key)
  {
    auto index = comboBoxPruner->findText(key.c_str());
    if (index<0)
      LOG_LINE << "Could not find pruner";
    else
      comboBoxPruner->setCurrentIndex(index);
  }

  /**
  */
  void TabPlanning::setInterpolator(const std::string& key)
  {
    auto index = comboBoxInterpolator->findText(key.c_str());
    if (index<0)
      LOG_LINE << "Could not find interpolator";
    else
      comboBoxInterpolator->setCurrentIndex(index);
  }
}
}