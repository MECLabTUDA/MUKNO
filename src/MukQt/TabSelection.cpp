#include "private/muk.pch"
#include "TabSelection.h"

#include "private/ParameterNet.h"
#include "muk_qt_tools.h"

#include "MukCommon/ICollisionDetector.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukObstacle.h"
#include "MukCommon/MukScene.h"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/qslider.h>
#include <QtWidgets/QSpinBox>
#include <qevent.h>
#include <qlayoutitem.h>
#include <QWheelEvent>
#include <qstyle.h>

#include <boost/format.hpp>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace
{
  static const char* OverlayNameOff = "Show CT-Overlay";
  static const char* OverlayNameOn = "Hide CT-Overlay";
  static const char* AdvancedOptionsWindowOff = "Advanced Options - Window";
  static const char* AdvancedOptionsEnlargeOff = "Advanced Options - Enlarge";
  static const char* AdvancedOptionsOn = "Hide Advanced Options";
  static const char* asObstacleOff = "Set as Obstacle";
  static const char* asObstacleOn = "Remove Obstacle";

  const double componentWeightsInitValue = 0.25;
  const double componentFiltersInitValue = 0;
  const double obstacleWeightsInitValue = 0.25;
  const double obstacleFiltersInitValue = 0;
  const double filterEndInitValue = 999;

  unsigned int mpComponentCount = 6;
  unsigned int mpComponentCountNoCT = 4;
  unsigned int mpObstacleCount = 12;
}

namespace
{
  using namespace gris::muk;

  /** special class for the QSlider, to allow doubleclicking in order to change the position
  */
  class MukSlider : public QSlider
  {
  public:
    MukSlider(Qt::Orientation orientation, QWidget *parent = 0);

  public:
    void mouseDoubleClickEvent(QMouseEvent *clickEvent);
  };

  MukSlider::MukSlider(Qt::Orientation orientation, QWidget *parent) : QSlider(orientation, parent)
  {
  }
  // when the slider is doubleclicked the bar jumps to that point
  void MukSlider::mouseDoubleClickEvent(QMouseEvent *clickEvent)
  {
    setValue(maximum() * clickEvent->pos().x() / (width() - 1));
  }
}

namespace gris
{
namespace muk
{
  /**
  */
  TabSelection::TabSelection(QWidget *parent)
    : QWidget(parent)
  {
    mpLayoutLarge = new QGridLayout();
    mpLayoutSmall = new QGridLayout();

    auto* motherLayout = new QGridLayout(this);
    motherLayout->setObjectName("motherLayout");

    auto* viewComponent = new QGraphicsView(this);
    mpComponentNet = new ParameterNet(300, 300, this, viewComponent);
    {
      std::vector<QColor> weightingColors = { QColor(0, 51, 117), QColor(161, 217, 244), QColor(0, 51, 117), QColor(161, 217, 244), QColor(0, 51, 117), QColor(161, 217, 244) };
      std::vector<double> values = { 0.25, 0.25, 0.25, 0.25, 0.25, 0.25 };
      std::vector<QString> names = { "minimum distance","sum of curvatures", "angle difference", "length", "bone thickness", "longest airhole" };
      std::vector<bool> isActive = { true,true,true,true,true,true };
      int n = mpComponentCount;
      mpComponentNet->setObjectName("Graphic Scene Component");
      mpComponentNet->setValues(values);
      mpComponentNet->setColors(weightingColors);
      mpComponentNet->setNames(names);
      mpComponentNet->setActiveObstacles(isActive);
      mpComponentNet->setN(n);
      mpComponentNet->setIsObstacleNet(false);
      mpComponentNet->reload();

      viewComponent->setObjectName("Graphic View Component");
      viewComponent->setRenderHint(QPainter::Antialiasing);
      viewComponent->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
      viewComponent->setBackgroundBrush(Qt::white);
      viewComponent->setScene(mpComponentNet);
      viewComponent->setToolTip("Adjust the weights of the different components.\nThe higher the value the more important will the component be when weighing");
    }

    // contains the Labels, the ComponentNet, the weighing Buttons, the weights for the categories and the Advanced Options Buttons
    {
      auto* label = new QLabel(this);
      label->setText("<b>component weights:");
      label->setObjectName("WeightLabelSmall");
      label->setTextFormat(Qt::RichText);
      mpLayoutSmallCollector.push_back(label);
      mpLayoutSmall->addWidget(label, 0, 0, 1, 9);

      label = new QLabel(this);
      label->setText("<b><center><font color=#C1973F size=6>2. weight the paths:");
      label->setObjectName("WeightLabelLarge");
      label->setTextFormat(Qt::RichText);
      mpLayoutLargeCollector.push_back(label);
      mpLayoutLarge->addWidget(label, 0, 19, 2, 24);

      mpLayoutSmall->addWidget(viewComponent, 1, 0, 13, 9);
      mpLayoutLarge->addWidget(viewComponent, 2, 19, 24, 18);

      auto* button = new QPushButton(this);
      button->setObjectName("EvaluatePathsButton");
      button->setToolTip("Adjust the weights of the different components.\nThe higher the value, the more important will the component be when weighing");
      button->setText("Evaluate Path");
      connect(button, &QPushButton::clicked, this, &TabSelection::evaluatePathClicked);
      mpLayoutSmallCollector.push_back(button);
      mpLayoutSmall->addWidget(button, 1, 9, 1, 3);

      button = new QPushButton(this);
      button->setObjectName("WeightPathsButton");
      button->setText("Weight Paths");
      button->setStyleSheet("QPushButton{font-size: 12pt }");
      button->setFixedHeight(50);
      button->setToolTip("Set weights of the different components and obstacles above and below, then click this.\nThe best path of the remaining paths after filtering will be selected according to the weighting");
      button->setIcon(QIcon("WeightIcon.png"));
      button->setIconSize(QSize(40, 40));
      connect(button, &QPushButton::clicked, this, &TabSelection::evaluatePathClicked);
      mpLayoutLargeCollector.push_back(button);
      mpLayoutLarge->addWidget(button, 26, 19, 4, 24);

      for (unsigned int i(0); i < mpComponentCount; i++)
      {
        auto* label = new QLabel(this);
        label->setText(mpComponentNet->getName(i) + ":");
        label->setObjectName(QString("CName %1").arg(i));
        mpLayoutSmall->addWidget(label, 2 + 2 * i, 9, 1, 3);
        mpLayoutLarge->addWidget(label, 2 + 4 * i, 37, 2, 6);

        auto* doublespinbox = new QDoubleSpinBox(this);
        doublespinbox->setDecimals(2);
        doublespinbox->setSingleStep(0.01);
        doublespinbox->setValue(0.25);
        doublespinbox->setMaximum(1.0);
        doublespinbox->setObjectName(QString("CValue %1").arg(i));
        connect(doublespinbox, SELECT<double>::OVERLOAD_OF(&QDoubleSpinBox::valueChanged), [=](double d) { mpComponentNet->setValue(i, d); emit this->componentWeightingChanged(); mpComponentNet->reloadValues(); });
        mpLayoutSmall->addWidget(doublespinbox, 3 + 2 * i, 9, 1, 3);
        mpLayoutLarge->addWidget(doublespinbox, 4 + 4 * i, 37, 2, 6);
      }

      button = new QPushButton(this);
      button->setObjectName("WindowButton");
      button->setText(AdvancedOptionsWindowOff);
      connect(button, &QPushButton::clicked, this, &TabSelection::windowClicked);
      mpLayoutSmall->addWidget(button, 14, 0, 1, 5);
      mpLayoutLarge->addWidget(button, 31, 50, 2, 5);

      button = new QPushButton(this);
      button->setObjectName("EnlargeButton");
      button->setText(AdvancedOptionsEnlargeOff);
      connect(button, &QPushButton::clicked, this, &TabSelection::enlargeClicked);
      mpLayoutSmall->addWidget(button, 14, 7, 1, 5);
      mpLayoutLarge->addWidget(button, 31, 57, 2, 5);
    }
      
    auto* viewObstacle = new QGraphicsView(this);
    mpObstacleNet = new ParameterNet(300, 300, this, viewObstacle);
    {
      std::vector<QColor>   weightingColors = { Qt::darkBlue, Qt::darkCyan, Qt::darkGreen, Qt::darkMagenta, Qt::darkRed, Qt::darkYellow, Qt::blue, Qt::cyan, Qt::green, Qt::magenta, Qt::red, Qt::yellow };
      auto   values = std::vector<double>(mpObstacleCount, obstacleWeightsInitValue);
      std::vector<QString>  names = { "Obstacle1","Obstacle2","Obstacle3","Obstacle4","Obstacle5","Obstacle6","Obstacle7","Obstacle8","Obstacle9","Obstacle10","Obstacle11","Obstacle12", };
      std::vector<bool>     isActive = std::vector<bool>(mpObstacleCount, true);
      size_t n = mpObstacleCount;
      mpObstacleNet->setObjectName("Graphic Scene Obstacle");
      mpObstacleNet->setValues(values);
      mpObstacleNet->setColors(weightingColors);
      mpObstacleNet->setNames(names);
      mpObstacleNet->setActiveObstacles(isActive);
      mpObstacleNet->setN(n);
      mpObstacleNet->setIsObstacleNet(true);
      mpObstacleNet->reload();

      viewObstacle->setObjectName("Graphic View Obstacle");
      viewObstacle->setRenderHint(QPainter::Antialiasing);
      viewObstacle->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
      viewObstacle->setBackgroundBrush(Qt::white);
      viewObstacle->setScene(mpObstacleNet);
      viewObstacle->setToolTip("Adjust the weights of the different obstacle distances.\nThe higher the value, the more will the distance to the obstacle matter compared to the others");
    }

    // contains the title, ObstacleNet and the weigths for up to 12 obstacles
    {
      auto* label = new QLabel(this);
      label->setText("<b>Obstacle Weights:");
      label->setTextFormat(Qt::RichText);
      label->setObjectName("ObstacleWeightsLabel");
      mpLayoutLargeCollector.push_back(label);
      mpLayoutLarge->addWidget(label, 30, 19, 2, 24);

      mpLayoutLargeCollector.push_back(viewObstacle);
      mpLayoutLarge->addWidget(viewObstacle, 32, 19, 24, 18);
      for (unsigned int i(0); i < mpObstacleCount; i++)
      {
        auto* doublespinbox = new QDoubleSpinBox(this);
        doublespinbox->setSingleStep(0.01);
        doublespinbox->setDecimals(2);
        doublespinbox->setValue(0.25);
        doublespinbox->setMaximum(1);
        doublespinbox->setObjectName(QString("OValue %1").arg(i));
        connect(doublespinbox, SELECT<double>::OVERLOAD_OF(&QDoubleSpinBox::valueChanged), [=](double d) { mpObstacleNet->setValue(i, d);  emit this->obstacleWeightingChanged(); mpObstacleNet->reloadValues(); });
        mpLayoutLargeCollector.push_back(doublespinbox);
        mpLayoutLarge->addWidget(doublespinbox, 32 + 2 * i, 37, 2, 6);
      }
    }
    auto* spacer = new QSpacerItem(0, 0, QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    auto* boarder = new QGraphicsView(this);
    auto* boarderBackground = new QGraphicsScene(boarder);
    boarder->setMaximumWidth(10);
    boarder->setObjectName("BoarderLeft");
    boarder->setBackgroundBrush(Qt::gray);
    boarder->setScene(boarderBackground);
    boarder->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    mpLayoutLargeCollector.push_back(boarder);
    mpLayoutLarge->addWidget(boarder, 0, 18, 56, 1);

    auto* label = new QLabel(this);
    label->setText("<b><center><font color=#C3C3C3 size=6>1. filter the paths:");
    label->setObjectName("ComponentFilterLabel");
    label->setTextFormat(Qt::RichText);
    mpLayoutLargeCollector.push_back(label);
    mpLayoutLarge->addWidget(label, 0, 0, 2, 18);

    std::vector<std::string> legendBefore = { "", "max ", "max ","max path", "max average ", "" };
    std::vector<std::string> legendAfter = { " to all Obstacles in mm, currently:", ", currently:", " in degree, currently:"," in mm, currently:", " in grayscale-Values, currently:", " in mm, currently:" };

    // contains the Component Filters
    for (unsigned int i(0); i < mpComponentCount; i++)
    {
      label = new QLabel(this);
      label->setText((QString)legendBefore[i].c_str() + mpComponentNet->getName(i) + (QString)legendAfter[i].c_str());
      label->setObjectName(QString("CSliderName %1").arg(i));
      mpLayoutLargeCollector.push_back(label);
      mpLayoutLarge->addWidget(label, 2 + 4 * i, 0, 2, 9);

      label = new QLabel(this);
      label->setText(QString("0"));
      label->setObjectName(QString("CSliderValue %1").arg(i));
      mpLayoutLargeCollector.push_back(label);
      mpLayoutLarge->addWidget(label, 2 + 4 * i, 9, 2, 9);

      label = new QLabel(this);
      label->setText(QString("0"));
      label->setObjectName(QString("CSliderMin %1").arg(i));
      mpLayoutLargeCollector.push_back(label);
      mpLayoutLarge->addWidget(label, 4 + 4 * i, 0, 2, 1);

      auto* slider = new MukSlider(Qt::Horizontal, this);
      slider->setMinimum(0);
      slider->setMaximum(1000);
      slider->setSingleStep(1);
      slider->setPageStep(10);
      slider->setValue(0);
      slider->setStyleSheet((i+1) % 2 ?"QSlider::handle:horizontal {background: #003375}" : "QSlider::handle:horizontal {background: #A1D9F4}");
      slider->setObjectName(QString("CSlider %1").arg(i));
      connect(slider, &QSlider::valueChanged, this, [=] { emit this->componentFilterChanged(i); emit this->obstacleFilterChanged(-1); emit requestParetoFrontReset();});
      mpLayoutLargeCollector.push_back(slider);
      mpLayoutLarge->addWidget(slider, 4 + 4 * i, 1, 2, 16);

      label = new QLabel(this);
      label->setText(QString("999"));
      label->setObjectName(QString("CSliderMax %1").arg(i));
      mpLayoutLargeCollector.push_back(label);
      mpLayoutLarge->addWidget(label, 4 + 4 * i, 17, 2, 1);
    }

    label = new QLabel(this);
    label->setText("<b>Obstacle Filter, min Distance to each Obstacle in mm:");
    label->setTextFormat(Qt::RichText);
    label->setObjectName("ObstacleFilterLabel");
    mpLayoutLargeCollector.push_back(label);
    mpLayoutLarge->addWidget(label, 30, 0, 2, 18);

    // contains the Obstacle Filters and the Filter Button
    {
      for (unsigned int i(0); i < mpObstacleCount; i++)
      {
        label = new QLabel(this);
        label->setText(mpObstacleNet->getNames()[i] + ", current:");
        label->setObjectName(QString("OSliderName %1").arg(i));
        mpLayoutLargeCollector.push_back(label);
        mpLayoutLarge->addWidget(label, 32 + 2 * i, 0, 2, 5);

        label = new QLabel(this);
        label->setText(QString("0"));
        label->setObjectName(QString("OSliderValue %1").arg(i));
        mpLayoutLargeCollector.push_back(label);
        mpLayoutLarge->addWidget(label, 32 + 2 * i, 5, 2, 1);

        label = new QLabel(this);
        label->setText(QString("0"));
        label->setObjectName(QString("OSliderMin %1").arg(i));
        mpLayoutLargeCollector.push_back(label);
        mpLayoutLarge->addWidget(label, 32 + 2 * i, 7, 2, 1);

        auto* slider = new MukSlider(Qt::Horizontal, this);
        slider->setMinimum(0);
        slider->setMaximum(1000);
        slider->setSingleStep(1);
        slider->setPageStep(10);
        slider->setValue(0);
        slider->setStyleSheet((i + 1) % 2 ? "QSlider::handle:horizontal {background: #003375}" : "QSlider::handle:horizontal {background: #A1D9F4}");
        slider->setObjectName(QString("OSlider %1").arg(i));
        connect(slider, &QSlider::valueChanged, this, [=] { emit this->componentFilterChanged(-1); emit this->obstacleFilterChanged(i); emit requestParetoFrontReset();});
        mpLayoutLargeCollector.push_back(slider);
        mpLayoutLarge->addWidget(slider, 32 + 2 * i, 8, 2, 9);

        label = new QLabel(this);
        label->setText(QString("999"));
        label->setObjectName(QString("OSliderMax %1").arg(i));
        mpLayoutLargeCollector.push_back(label);
        mpLayoutLarge->addWidget(label, 32 + 2 * i, 17, 2, 1);
      }
      auto* button = new QPushButton(this);
      button->setText("Filter Paths");
      button->setFixedHeight(50);
      button->setIcon(QIcon("FilterIcon.png"));
      button->setIconSize(QSize(40, 40));
      button->setStyleSheet("QPushButton{font-size: 12pt; }");
      button->setToolTip("Set requirements a path has to meet above and below, then click this.\nAll paths that don't satisfy the filters will be excluded from future computation.");
      button->setObjectName("FilterPathsButton");
      connect(button, &QPushButton::clicked, this, &TabSelection::filterPathsClicked);
      mpLayoutLargeCollector.push_back(button);
      mpLayoutLarge->addWidget(button, 26, 0, 4, 18);
    }

    spacer = new QSpacerItem(0, 0, QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    boarder = new QGraphicsView(this);
    boarderBackground = new QGraphicsScene(boarder);
    boarder->setMaximumWidth(10);
    boarder->setObjectName("BoarderRight");
    boarder->setBackgroundBrush(Qt::gray);
    boarder->setScene(boarderBackground);
    boarder->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    mpLayoutLargeCollector.push_back(boarder);
    mpLayoutLarge->addWidget(boarder, 0, 49, 56, 1);

    label = new QLabel(this);
    label->setText("<b>Quick-Select Buttons:");
    label->setTextFormat(Qt::RichText);
    mpLayoutSmall->addWidget(label, 15, 0, 1, 12);
    mpLayoutLarge->addWidget(label, 0, 50, 2, 12);

    //contains the Select Buttons
    {
      auto* button = new QPushButton(this);
      button->setObjectName("Select 0");
      button->setText("Select Largest Distance");
      connect(button, &QPushButton::clicked, this, &TabSelection::selectLargestDistanceClicked);
      mpLayoutSmall->addWidget(button, 16, 0, 1, 6);
      mpLayoutLarge->addWidget(button, 2, 50, 2, 6);

      button = new QPushButton(this);
      button->setObjectName("Select 1");
      button->setText("Select Straightest Path");
      connect(button, &QPushButton::clicked, this, &TabSelection::selectStraightestPathClicked);
      mpLayoutSmall->addWidget(button, 16, 6, 1, 6);
      mpLayoutLarge->addWidget(button, 2, 56, 2, 6);

      button = new QPushButton(this);
      button->setObjectName("Select 2");
      button->setText("Select Smallest Goal Angle");
      connect(button, &QPushButton::clicked, this, &TabSelection::selectBestAnglePathClicked);
      mpLayoutSmall->addWidget(button, 17, 0, 1, 6);
      mpLayoutLarge->addWidget(button, 4, 50, 2, 6);

      button = new QPushButton(this);
      button->setObjectName("Select 3");
      button->setText("Select Shortest Path");
      connect(button, &QPushButton::clicked, this, &TabSelection::selectShortestPathClicked);
      mpLayoutSmall->addWidget(button, 17, 6, 1, 6);
      mpLayoutLarge->addWidget(button, 4, 56, 2, 6);

      button = new QPushButton(this);
      button->setObjectName("Select 4");
      button->setText("Select Least Thick Bone");
      connect(button, &QPushButton::clicked, this, &TabSelection::selectLeastThickBoneClicked);
      mpLayoutSmall->addWidget(button, 18, 0, 1, 6);
      mpLayoutLarge->addWidget(button, 6, 50, 2, 6);

      button = new QPushButton(this);
      button->setObjectName("Select 5");
      button->setText("Select Shortest Air Hole");
      connect(button, &QPushButton::clicked, this, &TabSelection::selectShortestAirHoleClicked);
      mpLayoutSmall->addWidget(button, 18, 6, 1, 6);
      mpLayoutLarge->addWidget(button, 6, 56, 2, 6);
    }

    label = new QLabel(this);
    label->setText("<b>Path-Selection Buttons:");
    label->setTextFormat(Qt::RichText);
    mpLayoutSmall->addWidget(label, 19, 0, 1, 12);
    mpLayoutLarge->addWidget(label, 8, 50, 2, 12);

    // contains path Selection and Coloration
    {
      auto* button = new QPushButton(this);
      button->setObjectName("ShowSelectedOnlyButton");
      button->setText("Show Only Selected Path");
      connect(button, &QPushButton::clicked, this, &TabSelection::showOnlyClicked);
      mpLayoutSmall->addWidget(button, 20, 0, 1, 6);
      mpLayoutLarge->addWidget(button, 10, 50, 2, 6);

      auto* label = new QLabel(this);
      label->setObjectName("SelectPathLabel");
      label->setText("Select Path:");
      mpLayoutSmall->addWidget(label, 20, 7, 1, 2);
      mpLayoutLarge->addWidget(label, 10, 57, 2, 2);

      mpSelectedPath = new QSpinBox(this);
      mpSelectedPath->setMinimum(-1);
      mpSelectedPath->setMaximum(std::numeric_limits<int>::max());
      connect(mpSelectedPath, SELECT<int>::OVERLOAD_OF(&QSpinBox::valueChanged), [&](int i) { emit this->singlePathSelectionChanged(i); });
      mpLayoutSmall->addWidget(mpSelectedPath, 20, 9, 1, 3);
      mpLayoutLarge->addWidget(mpSelectedPath, 10, 59, 2, 3);

      button = new QPushButton(this);
      button->setObjectName("ResetButton");
      button->setText("Reset Selection");
      connect(button, &QPushButton::clicked, this, &TabSelection::resetSelectionClicked);
      mpLayoutSmall->addWidget(button, 21, 0, 1, 5);
      mpLayoutLarge->addWidget(button, 12, 50, 2, 5);

      button = new QPushButton(this);
      button->setObjectName("ColorPathsButton");
      button->setText("Color Paths");
      connect(button, &QPushButton::clicked, this, &TabSelection::colorPathsClicked);
      mpLayoutSmall->addWidget(button, 21, 7, 1, 5);
      mpLayoutLarge->addWidget(button, 12, 57, 2, 5);
    }

    label = new QLabel(this);
    label->setText("<b>CT-Overlay Buttons:");
    label->setTextFormat(Qt::RichText);
    mpLayoutSmall->addWidget(label, 22, 0, 1, 12);
    mpLayoutLarge->addWidget(label, 14, 50, 2, 12);

    //contains CT-Overlay
    {
      auto* button = new QPushButton(this);
      button->setObjectName("CT-OverlayButton");
      button->setText(OverlayNameOff);
      connect(button, &QPushButton::clicked, this, &TabSelection::ctOverlayClicked);
      mpLayoutSmall->addWidget(button, 23, 0, 1, 6);
      mpLayoutLarge->addWidget(button, 16, 50, 2, 6);

      auto* label = new QLabel(this);
      label->setObjectName("SelectStateLabel");
      label->setText("Select State:");
      mpLayoutSmall->addWidget(label, 23, 7, 1, 2);
      mpLayoutLarge->addWidget(label, 16, 57, 2, 2);

      mpSelectedState = new QSpinBox(this);
      mpSelectedState->setMinimum(0);
      mpSelectedState->setMaximum(std::numeric_limits<int>::max());
      connect(mpSelectedState, SELECT<int>::OVERLOAD_OF(&QSpinBox::valueChanged), [&](int i) { emit this->displayedStateChanged(i); });
      mpLayoutSmall->addWidget(mpSelectedState, 23, 9, 1, 3);
      mpLayoutLarge->addWidget(mpSelectedState, 16, 59, 2, 3);
    }

    label = new QLabel(this);
    label->setText("<b>Access-Canal Buttons:");
    label->setTextFormat(Qt::RichText);
    mpLayoutSmall->addWidget(label, 24, 0, 1, 12);
    mpLayoutLarge->addWidget(label, 18, 50, 2, 12);

    //contains the Access Canals, Auto-Fill Button
    {
      for (int i(0); i < 3; ++i)
      {
        label = new QLabel(this);
        label->setObjectName(QString("AccessCanalLabel %1").arg(i));
        label->setText((boost::format("Access Canal %d:") % (i + 1)).str().c_str());
        mpLayoutSmall->addWidget(label, 25 + i, 0, 1, 3);
        mpLayoutLarge->addWidget(label, 20 + 2 * i, 50, 2, 3);

        mAccessCanals[i] = new QLabel(this);
        mAccessCanals[i]->setAlignment(Qt::AlignCenter);
        mAccessCanals[i]->setText("");
        mpLayoutSmall->addWidget(mAccessCanals[i], 25 + i, 4, 1, 1);
        mpLayoutLarge->addWidget(mAccessCanals[i], 20 + 2 * i, 54, 2, 1);

        auto* pushButton = new QPushButton(this);
        pushButton->setObjectName(QString("AccessCanalButton %1").arg(i));
        pushButton->setText("Set");
        connect(pushButton, &QPushButton::clicked, [=]() { emit this->indexChosenClicked(i); });
        mpLayoutSmall->addWidget(pushButton, 25 + i, 5, 1, 2);
        mpLayoutLarge->addWidget(pushButton, 20 + 2 * i, 55, 2, 2);

        auto* spinbox = new QDoubleSpinBox(this);
        spinbox->setSingleStep(0.01);
        spinbox->setValue(1);
        spinbox->setObjectName(QString("cutOffDistanceSpinBox %1").arg(i));
        spinbox->setToolTip(QString("all states of accessCanal %1 that are closer\n to the goalpoint than this distance are marked blue").arg(i + 1));
        connect(spinbox, &QDoubleSpinBox::editingFinished, [=] { emit this->cutOffDistanceChanged(i, spinbox->value()); });
        mpLayoutSmall->addWidget(spinbox, 25 + i, 7, 1, 2);
        mpLayoutLarge->addWidget(spinbox, 20 + 2 * i, 57, 2, 2);


        auto* button = new QPushButton(this);
        button->setObjectName(QString("asObstacleButton %1").arg(i));
        button->setText(asObstacleOff);
        button->setToolTip(QString("Sets the not-blue States of AccessCanal %1 as an Obstacle").arg(i + 1));
        connect(button, &QPushButton::clicked, [=] { emit this->asObstacleClicked(i); });
        mpLayoutSmall->addWidget(button, 25 + i, 9, 1, 3);
        mpLayoutLarge->addWidget(button, 20 + 2 * i, 59, 2, 3);
      }

      auto* button = new QPushButton(this);
      button->setObjectName("AutoFillCanalsButton");
      button->setText("Auto-Fill the Canals");
      button->setToolTip("fills all unset Access Canals with the \none apart from each other the most");
      connect(button, &QPushButton::clicked, [&]() { emit fillCanalsClicked(); });
      mpLayoutSmall->addWidget(button, 28, 2, 1, 8);
      mpLayoutLarge->addWidget(button, 26, 52, 2, 8);
    }

    auto* button = new QPushButton(this);
    button->setObjectName("paretoFrontButton");
    button->setText("Show Pareto-Front Window");
    button->setToolTip("opens the window for the Pareto-Front");
    connect(button, &QPushButton::clicked, [&] () { emit paretoFrontClicked(); });
    mpLayoutSmall->addWidget(button, 29, 0, 1, 12);
    mpLayoutLarge->addWidget(button, 28, 50, 2, 12);

    spacer = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);
    mpLayoutSmall->addItem(spacer, 30, 0, 7, 12);
    mpLayoutLarge->addItem(spacer, 33, 50, 27, 12);

    setLayoutLarge(false);

    // Contains Layout for the ParetoWindow and it's widgets
    {
      //mpParetoWidget = new ParetoWidget(this);
      /*auto* paretoWindowLayout = new QGridLayout(this);
      paretoWindowLayout->setObjectName("paretoLayout");*/

      /*mpPossibleParameterList->append("- not selected -");
      mpPossibleParameterList->append("Path Length");
      mpPossibleParameterList->append("Minimum Distance");
      mpPossibleParameterList->append("Minimum Distance to Facialis");
      mpPossibleParameterList->append("Minimum Distance to Chorda");*/

      /*mFatherWidget = new QWidget();

      auto gridLayout = new QGridLayout(mFatherWidget);
      auto Hlayout = new QHBoxLayout();
      auto Vlayout = new QVBoxLayout();
      Vlayout->addLayout(Hlayout);
      gridLayout->addLayout(Vlayout, 0, 9, 8, 4);

      auto* viewPareto = new QGraphicsView(this);
      mpParetoFront = new ParetoFront(800, 800, viewPareto);
      mpParetoFront->setObjectName("ParetoFront");
      mpParetoFront->connect(mpParetoFront, &ParetoFront::clickedPath, this, [=](size_t ind) { mpSelectedPath->setValue(int(ind)); });
      viewPareto->setObjectName("Graphic View Pareto");
      viewPareto->setRenderHint(QPainter::Antialiasing);
      viewPareto->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
      viewPareto->setBackgroundBrush(Qt::white);
      viewPareto->setScene(mpParetoFront);
      viewPareto->setToolTip("Shows the ParetoFront for the chosen Parameters.");
      gridLayout->addWidget(viewPareto, 0, 0, 8, 8);

      auto* comboBox = new QComboBox(mFatherWidget);
      comboBox->setObjectName("paretoParam1");
      comboBox->addItems(*mpPossibleParameterList);
      comboBox->connect(comboBox, SELECT<const QString&>::OVERLOAD_OF(&QComboBox::currentIndexChanged), this, [&](const QString& str) { paramChosen(true, str); emit this->parameterChosen(true, str);});
      Hlayout->addWidget(comboBox);

      comboBox = new QComboBox(mFatherWidget);
      comboBox->setObjectName("paretoParam2");
      comboBox->addItems(*mpPossibleParameterList);
      comboBox->connect(comboBox, SELECT<const QString&>::OVERLOAD_OF(&QComboBox::currentIndexChanged), this, [&](const QString& str) { paramChosen(false, str); emit this->parameterChosen(false, str);});
      Hlayout->addWidget(comboBox);

      button = new QPushButton(this);
      button->setObjectName("paretoExitButton");
      button->setText("Close Window");
      button->setToolTip("closes the Window for the Pareto Front");
      button->connect(button, &QPushButton::clicked, this, &TabSelection::paretoExitClicked);
      Vlayout->addWidget(button);

      mFatherWidget->setLayout(gridLayout);*/
    }
  }

  /** Sets the Selection Tab layout to large, hiding all the items that are not in the large Collector
  */
  void TabSelection::setLayoutLarge(bool large)
  {
    auto* motherLayout = findChild<QGridLayout*>("motherLayout");
    if (large)
    {
      motherLayout->removeItem(mpLayoutSmall);
      Q_FOREACH(QWidget* widget, mpLayoutSmallCollector)
      {
          widget->hide();
      }
      motherLayout->addLayout(mpLayoutLarge, 0, 0);
      Q_FOREACH(QWidget* widget, mpLayoutLargeCollector)
      {
        widget->setHidden(false);
      }
    }
    else
    {
      motherLayout->removeItem(mpLayoutLarge);
      Q_FOREACH(QWidget* widget, mpLayoutLargeCollector)
      {
        widget->hide();
      }
      motherLayout->addLayout(mpLayoutSmall, 0, 0);
      Q_FOREACH(QWidget* widget, mpLayoutSmallCollector)
      {
        widget->setHidden(false);
      }
    }
  }


  /** updates the amount of Components depending on ctFileLoaded (still respects the state of the tab)
  */
  void TabSelection::updateComponentCount(bool ctFileLoaded, bool tabIsSmall)
  {
    auto* componentScene = findChild<ParameterNet*>("Graphic Scene Component");
    componentScene->setN(ctFileLoaded ? mpComponentCount : mpComponentCountNoCT);
    componentScene->reload();
    for (size_t i(mpComponentCountNoCT); i < mpComponentCount; ++i)
    {
      auto* label = findChild<QLabel*>(QString("CName %1").arg(i));
      auto* doublespinbox = findChild<QDoubleSpinBox*>(QString("CValue %1").arg(i));
      auto* button = findChild<QPushButton*>(QString("Select %1").arg(i));
      auto* sliderName = findChild<QLabel*>(QString("CSliderName %1").arg(i));
      auto* sliderValue = findChild<QLabel*>(QString("CSliderValue %1").arg(i));
      auto* sliderMin = findChild<QLabel*>(QString("CSliderMin %1").arg(i));
      auto* slider = findChild<QSlider*>(QString("CSlider %1").arg(i));
      auto* sliderMax = findChild<QLabel*>(QString("CSliderMax %1").arg(i));
      if (ctFileLoaded)
      {
        label->setHidden(false);
        doublespinbox->setHidden(false);
        button->setHidden(false);
        if (!tabIsSmall)
        {
          sliderName->setHidden(false);
          sliderValue->setHidden(false);
          sliderMin->setHidden(false);
          slider->setHidden(false);
          sliderMax->setHidden(false);
        }
      }
      else
      {
        label->hide();
        doublespinbox->hide();
        button->hide();
        sliderName->hide();
        sliderValue->hide();
        sliderMin->hide();
        slider->hide();
        sliderMax->hide();
      }
    }
  }

  /** updates the amount of active Obstacles with a maximum of 12 active Obstacles at the same time
  */
  void TabSelection::updateObstacleCount(bool tabIsSmall, std::vector<std::string> activeObstacleKeys, std::vector<QColor> colors)
  {
    auto* obstacleScene = findChild<ParameterNet*>("Graphic Scene Obstacle");
    obstacleScene->setN(activeObstacleKeys.size());
    for (size_t i(0); i < mpObstacleCount; i++)
    {
      auto* doubleSpinBox = findChild<QDoubleSpinBox*>(QString("OValue %1").arg(i));
      auto* sliderName = findChild<QLabel*>(QString("OSliderName %1").arg(i));
      auto* sliderValue = findChild<QLabel*>(QString("OSliderValue %1").arg(i));
      auto* sliderMin = findChild<QLabel*>(QString("OSliderMin %1").arg(i));
      auto* slider = findChild<QSlider*>(QString("OSlider %1").arg(i));
      auto* sliderMax = findChild<QLabel*>(QString("OSliderMax %1").arg(i));
      if (i < activeObstacleKeys.size()) // show those
      {
        if (!tabIsSmall)
        {
          doubleSpinBox->setHidden(false);
          sliderName->setHidden(false);
          sliderValue->setHidden(false);
          sliderMin->setHidden(false);
          slider->setHidden(false);
          sliderMax->setHidden(false);
        }
        sliderName->setText((QString)(activeObstacleKeys[i]).c_str() + ", currently:");
        obstacleScene->setActiveObstacle(i, true);
        obstacleScene->setName(i, (QString)(activeObstacleKeys[i]).c_str());
        obstacleScene->setColor(i, colors[i]);
      }
      else // hide the rest
      {
        doubleSpinBox->hide();
        sliderName->hide();
        sliderValue->hide();
        sliderMin->hide();
        slider->hide();
        sliderMax->hide();
        obstacleScene->setActiveObstacle(i, false);
      }
    }
    obstacleScene->reload();
  }

  /** updates the Values of the Component Weights
  */
  void TabSelection::updateComponentWeights(std::vector<double> weights)
  {
    auto* componentScene = findChild<ParameterNet*>("Graphic Scene Component");
    for (size_t i(0); i < weights.size(); i++)
    {
      auto* doublespinbox = findChild<QDoubleSpinBox*>(QString("CValue %1").arg(i));
      const QSignalBlocker blocker(doublespinbox);
      doublespinbox->setValue(weights[i]);
      componentScene->setValue(i, weights[i]);
    }
    componentScene->reloadValues();
  }

  /** updates the Values of the Obstacle Weights
  */
  void TabSelection::updateObstacleWeights(std::vector<double> weights)
  {
    auto* obstacleScene = findChild<ParameterNet*>("Graphic Scene Obstacle");
    for (size_t i(0); i < weights.size(); i++)
    {
      auto* doublespinbox = findChild<QDoubleSpinBox*>(QString("OValue %1").arg(i));
      const QSignalBlocker blocker(doublespinbox);
      doublespinbox->setValue(weights[i]);
      obstacleScene->setValue(i, weights[i]);
    }
    obstacleScene->reloadValues();
  }

  /** updates the Values of the ComponentFilters
  */
  void TabSelection::updateComponentFilter(std::vector<double> sliderValue, std::vector<double> filterValue, std::vector<double> filterMin, std::vector<double> filterMax)
  {
    for (size_t i(0); i < mpComponentCount; i++)
    {
      auto* slider = findChild<QSlider*>(QString("CSlider %1").arg(i));
      const QSignalBlocker blocker(slider);
      slider->setValue(sliderValue[i]);
      auto* sliderValue = findChild<QLabel*>(QString("CSliderValue %1").arg(i));
      sliderValue->setText(QString("%1").arg(roundDouble(filterValue[i],2)));
      auto* sliderMin = findChild<QLabel*>(QString("CSliderMin %1").arg(i));
      sliderMin->setText(QString("%1").arg(roundDouble(filterMin[i],2)));
      auto* sliderMax = findChild<QLabel*>(QString("CSliderMax %1").arg(i));
      sliderMax->setText(QString("%1").arg(roundDouble(filterMax[i],2)));
    }
  }

  /** updates the Values of the ObstacleFilters
  */
  void TabSelection::updateObstacleFilter(std::vector<double> sliderValue, std::vector<double> filterValue, std::vector<double> filterMin, std::vector<double> filterMax)
  {
    for (size_t i(0); i < filterValue.size(); i++)
    {
      auto* slider = findChild<QSlider*>(QString("OSlider %1").arg(i));
      const QSignalBlocker blocker(slider);
      slider->setValue(sliderValue[i]);
      auto* sliderValue = findChild<QLabel*>(QString("OSliderValue %1").arg(i));
      sliderValue->setText(QString("%1").arg(roundDouble(filterValue[i], 2)));
      auto* sliderMin = findChild<QLabel*>(QString("OSliderMin %1").arg(i));
      sliderMin->setText(QString("%1").arg(roundDouble(filterMin[i], 2)));
      auto* sliderMax = findChild<QLabel*>(QString("OSliderMax %1").arg(i));
      sliderMax->setText(QString("%1").arg(roundDouble(filterMax[i], 2)));
    }
  }

  /** updates all Weights and Filters with the Init Values
  */
  void TabSelection::resetTabSelection()
  {
    updateComponentWeights(std::vector<double>(mpComponentCount, componentWeightsInitValue));
    updateObstacleWeights(std::vector<double>(mpObstacleCount, obstacleWeightsInitValue));
    auto tempComponentInitVector = std::vector<double>(mpComponentCount, componentFiltersInitValue);
    updateComponentFilter(tempComponentInitVector, tempComponentInitVector, tempComponentInitVector, std::vector<double>(mpComponentCount, filterEndInitValue));
    auto tempObstacleInitVector = std::vector<double>(mpObstacleCount, obstacleFiltersInitValue);
    updateObstacleFilter(tempObstacleInitVector, tempObstacleInitVector, tempObstacleInitVector, std::vector<double>(mpObstacleCount, filterEndInitValue));
    emit requestParetoFrontReset();
  }

  /** returns the Values of all possible (6) ComponentWeights
  */
  std::vector<double> TabSelection::getComponentWeights() const
  {
    return mpComponentNet->getValues();
  }

  /** returns the Values of all possible (6) ComponentFilters
  */
  std::vector<double> TabSelection::getComponentFilter() const
  {
    auto ret = std::vector<double>();
    for (size_t i(0); i < mpComponentCount; i++)
    {
      auto* slider = findChild<QSlider*>(QString("CSlider %1").arg(i));
      ret.push_back(slider->value());
    }
    return ret;
  }

  /** returns the Values of all possible (12) ObstacleWeights
  */
  std::vector<double> TabSelection::getObstacleWeights() const
  {
    return mpObstacleNet->getValues();
  }

  /** returns the Values of all possible (12) ObstacleFilters
  */
  std::vector<double> TabSelection::getObstacleFilter() const
  {
    auto ret = std::vector<double>();
    for (size_t i(0); i < mpObstacleCount; i++)
    {
      auto* slider = findChild<QSlider*>(QString("OSlider %1").arg(i));
      ret.push_back(slider->value());
    }
    return ret;
  }

  /** return the goalThreshold distance set in the TabSelection for the canal at canalInd
  */
  double TabSelection::getCutOffDistance(size_t canalInd)
  {
    auto spinbox = findChild<QDoubleSpinBox*>(QString("cutOffDistanceSpinBox %1").arg(canalInd));
    return spinbox->value();
  }

  /** \brief Toggles the text of the CT-Overlay button according to the status
  */
  void TabSelection::toggleCTOverlay(bool on)
  {
    auto* button = findChild<QPushButton*>("CT-OverlayButton");
    if (on)
      button->setText(OverlayNameOn);
    else
      button->setText(OverlayNameOff);
  }

  /** \brief Toggles the text of the AdvancedOptions-Window button according to the status
  */
  void TabSelection::toggleAdvancedOptionsWindow(bool on)
  {
    auto* button = findChild<QPushButton*>("WindowButton");
    if (on)
      button->setText(AdvancedOptionsOn);
    else
      button->setText(AdvancedOptionsWindowOff);
  }

  /** \brief Toggles the text of the AdvancedOptions-Enlarge button according to the status
  */
  void TabSelection::toggleAdvancedOptionsEnlarge(bool on)
  {
    auto* button = findChild<QPushButton*>("EnlargeButton");
    if (on)
      button->setText(AdvancedOptionsOn);
    else
      button->setText(AdvancedOptionsEnlargeOff);
  }

  /** returns the path selecting by the spinbox
  */
  size_t TabSelection::getSinglePathSelection() const
  {
    return static_cast<size_t>(mpSelectedPath->value());
  }

  /** updates the spinbox with the selected path (changes through evaluate path for example)
  */
  void TabSelection::setSinglePathSelection(size_t i)
  {
    const QSignalBlocker blocker(mpSelectedPath);
    mpSelectedPath->setValue(static_cast<int>(i));
  }

  /** sets the number of paths
  */
  void TabSelection::setMaxPathIndex(int idx)
  {
    mpSelectedPath->setMaximum(idx);
  }

  /** \brief Updates the Select State Spinbox when the Value is changed through scrolling with the CT-Overlay
  */
  void TabSelection::updateDisplayState(size_t i, size_t max)
  {
    const QSignalBlocker blocker(mpSelectedState);
    mpSelectedState->setValue(static_cast<int>(i));
    mpSelectedState->setMaximum(static_cast<int>(max));
  }

  /**
  Sends the Signal of a scroll to SelectionLogic
  */
  void TabSelection::wheelEvent(QWheelEvent *event)
  {
    emit this->ctOverlayScrolled(event->delta() > 0 ? true : false);
  }

  /** \brief sets the label of access canal 1,2 or 3 to specified path idx

    resets label to empty string if pathIdx is invalid (<0)
  */
  void TabSelection::setAccessCanalIndex(int canalIdx, int pathIdx)
  {
    if (canalIdx < 0 || canalIdx>2)
      return;
    if (pathIdx < 0)
    {
      mAccessCanals[canalIdx]->setText("");
    }
    else
    {
      mAccessCanals[canalIdx]->setText(std::to_string(pathIdx).c_str());
    }
  }

  /** \brief controls if the AccessCanals and their buttons have the same idx as the singlePathSpinbox
      Shows the 'Unset' text if true, else the 'Set' text
  */
  void TabSelection::checkAccessCanals(std::vector<size_t> chosenIdx)
  {
    auto currentPath = mpSelectedPath->value();
    for (size_t i(0); i < chosenIdx.size(); i++)
    {
      auto canalButton = findChild<QPushButton*>(QString("AccessCanalButton %1").arg(i));
      if (chosenIdx[i] == currentPath && chosenIdx[i] != -1)
        canalButton->setText("Unset");
      else
        canalButton->setText("Set");
    }
  }

  /** toggles the SetAsObstacle button corresponding to on and disables interaction with
      the accessCanalButton and the cutOffDistanceSpinbox if on is true
  */
  void TabSelection::toggleAsObstacle(bool on, size_t canalInd)
  {
    auto button = findChild<QPushButton*>(QString("asObstacleButton %1").arg(canalInd));
    button->setText(on ? asObstacleOn : asObstacleOff);
    button = findChild<QPushButton*>(QString("AccessCanalButton %1").arg(canalInd));
    button->setDisabled(on);
    auto spinbox = findChild<QDoubleSpinBox*>(QString("cutOffDistanceSpinBox %1").arg(canalInd));
    spinbox->setDisabled(on);
  }

  /** updates the cutOffDistanceSpinBox of the canal canalInd with distance as the value
  */
  void TabSelection::updateCutOffDistance(size_t canalInd, double distance)
  {
    auto spinbox = findChild<QDoubleSpinBox*>(QString("cutOffDistanceSpinBox %1").arg(canalInd));
    const QSignalBlocker blocker(spinbox);
    spinbox->setValue(distance);
  }
    
  /** disables the paretoWindowButton aslong as the window is open
  */
  void TabSelection::paretoStatus(const bool is_active) const
  {
    auto button = findChild<QPushButton*>("paretoFrontButton");
    button->setDisabled(is_active);
  }

  /** \utility function to return a double cut of after a given amount of decimals
  */
  double TabSelection::roundDouble(double value, size_t decimals)
  {
    return (double)((int)(value * pow(10, decimals))) / pow(10, decimals);
  }
}
}