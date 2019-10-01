#include "private/muk.pch"
#include "ProblemDefinitionWidget.h"

#include "MukCommon/MukException.h"

#include <QGridLayout.h>
#include <qlayout.h>
#include <qlabel.h>
#include <qlineedit.h>
#include <qtoolbutton.h>

#include <boost/format.hpp>

namespace gris
{
namespace muk
{
  /**
  */
  ProblemDefinitionWidget::ProblemDefinitionWidget(QWidget* parent)
    : QWidget(parent)
    , mNumDecimals(3)
    , mpToolButtonAddStart(new QToolButton(this))
    , mpToolButtonAddGoal(new QToolButton(this))
    , mpToolButtonAddWaypoint(new QToolButton(this))
    , mpToolButtonDeleteRegion(new QToolButton(this))
    , mpToolButtonUndo(new QToolButton(this))
    , mpToolButtonReverseWaypoints(new QToolButton(this))
  {
    auto* pLayout = new QVBoxLayout(this);
    pLayout->setContentsMargins(0, 0, 0, 0); // makes sure the Widget takes space of the whole parent widget

    mpTreeWidgetWaypointVis = new QTreeWidget(this);
    {
      mpTreeWidgetWaypointVis->setObjectName("treeWidgetWaypoints");
      mpTreeWidgetWaypointVis->setEnabled(true);
      auto* pHeaderItem = mpTreeWidgetWaypointVis->headerItem();
      pHeaderItem->setText(0, "State Regions");
      pHeaderItem->setText(1, "Position");
      pHeaderItem->setText(2, "Direction");
      pLayout->addWidget(mpTreeWidgetWaypointVis);

      // assume 3 rows for the above headers + 3 actual state regions -> minimal size is 3 header + 3 header + header line = 7
      // something like 7 * pHeaderItem->sizeHint().height(), but that doesn't work
      // below is a working hack
      mpTreeWidgetWaypointVis->setMinimumHeight(5 * mpTreeWidgetWaypointVis->size().height());
      mpTreeWidgetWaypointVis->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred));
    }

    auto* pButtonLayout = new QGridLayout();
    {
      QIcon icon;

      mpToolButtonAddStart->setText("Add Start Region");
      mpToolButtonAddStart->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
      icon.addFile("../resources/icons/windows/Add.png", QSize(), QIcon::Normal, QIcon::Off);
      mpToolButtonAddStart->setIcon(icon);
      mpToolButtonAddStart->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
      connect(mpToolButtonAddStart, &QToolButton::clicked, this, &ProblemDefinitionWidget::addStartRegionClicked);
      pButtonLayout->addWidget(mpToolButtonAddStart, 0, 0);

      mpToolButtonAddWaypoint->setText("Add Waypoint");
      mpToolButtonAddWaypoint->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
      mpToolButtonAddWaypoint->setIcon(icon);
      mpToolButtonAddWaypoint->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
      //connect(mpToolButtonAddWaypoint, &QToolButton::clicked, this, &ProblemDefinitionWidget::addWaypointClicked);
      mpToolButtonAddWaypoint->setDisabled(true);
      pButtonLayout->addWidget(mpToolButtonAddWaypoint, 0, 1);

      mpToolButtonAddGoal->setText("Add Goal Region");
      mpToolButtonAddGoal->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
      mpToolButtonAddGoal->setIcon(icon);
      mpToolButtonAddGoal->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
      connect(mpToolButtonAddGoal, &QToolButton::clicked, this, &ProblemDefinitionWidget::addGoalRegionClicked);
      pButtonLayout->addWidget(mpToolButtonAddGoal, 0, 2);

      mpToolButtonDeleteRegion->setText("Delete Selected Region");
      mpToolButtonDeleteRegion->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
      icon.addFile("../resources/icons/windows/Delete.png", QSize(), QIcon::Normal, QIcon::Off);
      mpToolButtonDeleteRegion->setIcon(icon);
      mpToolButtonDeleteRegion->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
      connect(mpToolButtonDeleteRegion, &QToolButton::clicked, this, &ProblemDefinitionWidget::deleteRegionClicked);
      pButtonLayout->addWidget(mpToolButtonDeleteRegion, 1, 0);
      
      mpToolButtonUndo->setText("Undo");
      mpToolButtonUndo->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
      icon.addFile("../resources/icons/windows/Undo.png", QSize(), QIcon::Normal, QIcon::Off);
      mpToolButtonUndo->setIcon(icon);
      mpToolButtonUndo->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
      mpToolButtonUndo->setDisabled(true);
      //connect(mpToolButtonUndo, &QToolButton::clicked, this, &ProblemDefinitionWidget::undoClicked);
      pButtonLayout->addWidget(mpToolButtonUndo, 1, 1);

      mpToolButtonReverseWaypoints->setText("Reverse");
      mpToolButtonReverseWaypoints->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred));
      icon.addFile("../resources/icons/windows/Reverse.png", QSize(), QIcon::Normal, QIcon::Off);
      mpToolButtonReverseWaypoints->setIcon(icon);
      mpToolButtonReverseWaypoints->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
      connect(mpToolButtonReverseWaypoints, &QToolButton::clicked, this, &ProblemDefinitionWidget::reverseClicked);
      //mpToolButtonReverseWaypoints->setDisabled(true);
      pButtonLayout->addWidget(mpToolButtonReverseWaypoints, 1, 2);

      pLayout->addLayout(pButtonLayout);
    }

    //// layout line edits, sliders and labels
    //auto* pSliderLayout = new QGridLayout();
    //{
    //  auto* label = new QLabel(this);
    //  {
    //    auto labelSizePolicy = QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    //    label->setObjectName("labelX");
    //    label->setText("X");
    //    label->setSizePolicy(labelSizePolicy);
    //    //label->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
    //    pSliderLayout->addWidget(label, 0, 0, 1, 1);

    //    label = new QLabel(this);
    //    label->setObjectName("labelY");
    //    label->setText("Y");
    //    label->setSizePolicy(labelSizePolicy);
    //    pSliderLayout->addWidget(label, 0, 2, 1, 1);

    //    label = new QLabel(this);
    //    label->setObjectName("labelZ");
    //    label->setText("Z");
    //    label->setSizePolicy(labelSizePolicy);
    //    pSliderLayout->addWidget(label, 0, 4, 1, 1);
    //  }
    //  {
    //    QString text = "100.000"; // sample text with precision        
    //    QFont font("", 0); //use QFontMetrics this way;
    //    QFontMetrics fm(font);
    //    int pixelsWide = fm.width(text);
    //    int pixelsHigh = fm.height();

    //    auto lineEditSizePolicy = QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    //    mpLineEditX = new QLineEdit(this);
    //    mpLineEditX->setObjectName("lineEditX");
    //    mpLineEditX->setFixedSize(pixelsWide, pixelsHigh);
    //    mpLineEditX->setSizePolicy(lineEditSizePolicy);
    //    connect(mpLineEditX, &QLineEdit::returnPressed, [&] () { emit this->updateXClicked(std::stod(mpLineEditX->text().toLocal8Bit().constData()));});
    //    pSliderLayout->addWidget(mpLineEditX, 0, 1, 1, 1);

    //    mpLineEditY = new QLineEdit(this);
    //    mpLineEditY->setObjectName("lineEditY");
    //    mpLineEditY->setFixedSize(pixelsWide, pixelsHigh);
    //    mpLineEditY->setSizePolicy(lineEditSizePolicy);
    //    connect(mpLineEditY, &QLineEdit::returnPressed, [&] () { emit this->updateYClicked(std::stod(mpLineEditY->text().toLocal8Bit().constData()));});
    //    pSliderLayout->addWidget(mpLineEditY, 0, 3, 1, 1);

    //    mpLineEditZ = new QLineEdit(this);
    //    mpLineEditZ->setObjectName("lineEditZ");
    //    mpLineEditZ->setFixedSize(pixelsWide, pixelsHigh);
    //    mpLineEditZ->setSizePolicy(lineEditSizePolicy);
    //    connect(mpLineEditZ, &QLineEdit::returnPressed, [&] () { emit this->updateZClicked(std::stod(mpLineEditZ->text().toLocal8Bit().constData()));});
    //    pSliderLayout->addWidget(mpLineEditZ, 0, 5, 1, 1);
    //  }

    //  pLayout->addLayout(pSliderLayout);
    //}

    auto* spacer = new QSpacerItem(0, 40, QSizePolicy::Minimum, QSizePolicy::MinimumExpanding); 
    pLayout->addItem(spacer);

    mpDirectionWidget = new SphereInteraction(this);
    {
      mpDirectionWidget->setObjectName("DirectionWidget");
      pLayout->addWidget(mpDirectionWidget);
    }
    mpDirectionWidget->initialize();
  }

  /**
  */
  ProblemDefinitionWidget::~ProblemDefinitionWidget()
  {
  }

  /**
  */
  ProblemDefinitionWidget::EnRegionType_t ProblemDefinitionWidget::getSelectedType() const
  {
    // check if a valid widgetItem was selected
    const auto& list = mpTreeWidgetWaypointVis->selectedItems();
    if (list.size() != 1)
      return EnRegionType_t::enNone;
    auto* pItem = list.back();
    auto pParent = pItem->parent();
    if (nullptr == pParent)
      return EnRegionType_t::enNone;
    if (pParent->text(0) == "Start Regions")
    {
      return EnRegionType_t::enStart;
    }
    else if (pParent->text(0) == "Goal Regions")
    {
      return EnRegionType_t::enGoal;
    }
    else
    {
      return EnRegionType_t::enWaypoint;
    }
  }

  /**
  */
  int ProblemDefinitionWidget::getSelectedIndex() const
  {
    // check if a valid widgetItem was selected
    const auto& list = mpTreeWidgetWaypointVis->selectedItems();
    if (list.size() != 1)
      return -1;
    auto* pItem = list.back();
    auto pParent = pItem->parent();
    if (nullptr == pParent)
      return -1;
    return pParent->indexOfChild(pItem);
  }

  /**
  */
  void ProblemDefinitionWidget::setSelection(EnRegionType_t type, int index)
  {
    QSignalBlocker blocker(this);
    mpTreeWidgetWaypointVis->clearSelection();
    if (type == EnRegionType_t::enNone)
      return;
    auto* pItem = mpTreeWidgetWaypointVis->topLevelItem(type);
    const int N = pItem->childCount();
    if (index > N)
    {
      std::string info = "ProblemDefinitionWidget has not enough children of this type: " + std::to_string(type) 
        + ", children: " + std::to_string(N) + ", requested index: " + std::to_string(index);
      throw MUK_EXCEPTION("Index out of bounds.", info.c_str());
    }
    auto* pChild = pItem->child(index);
    mpTreeWidgetWaypointVis->setItemSelected(pChild, true);
  }

  /**
  */
  void ProblemDefinitionWidget::setState(const MukState& state)
  {
    setPosition(state.coords);
    setDirection(state.tangent);
  }

  /**
  */
  void ProblemDefinitionWidget::setPosition(const Vec3d& p)
  {
    const QSignalBlocker blocker(this);
      {
        //const QSignalBlocker blocker(mpLineEditX);
        //mpLineEditX->setText(QString::number(p.x(), 'd', mNumDecimals));;
      }
      {
        /*const QSignalBlocker blocker(mpLineEditY);
        mpLineEditY->setText(QString::number(p.y(), 'd', mNumDecimals));;*/
      }
      {
        /*const QSignalBlocker blocker(mpLineEditZ);
        mpLineEditZ->setText(QString::number(p.z(), 'd', mNumDecimals));;*/
      }
    auto items = mpTreeWidgetWaypointVis->selectedItems();
    if (items.size() != 1)
      return; // should probably throw
    auto* pItem = items.back();
    QString posString = QString::number(p.x(), 'd', mNumDecimals) + " " +  QString::number(p.y(), 'd', mNumDecimals) + " " +  QString::number(p.z(), 'd', mNumDecimals);
    pItem->setText(1, posString);
  }

  /**
  */
  void ProblemDefinitionWidget::setDirection(const Vec3d& q)
  {
    const QSignalBlocker blocker(this);
    auto items = mpTreeWidgetWaypointVis->selectedItems();
    if (items.size() != 1)
      return; // should probably throw
    auto* pItem = items.back();
    QString posString = QString::number(q.x(), 'd', mNumDecimals) + " " +  QString::number(q.y(), 'd', mNumDecimals) + " " +  QString::number(q.z(), 'd', mNumDecimals);
    pItem->setText(2, posString);
  }

  /**
  */
  void ProblemDefinitionWidget::clearState()
  {
    /*mpLineEditX->clear();
    mpLineEditY->clear();
    mpLineEditZ->clear();*/
  }
}
}