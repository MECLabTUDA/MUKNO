#include "private/muk.pch"
#include "ParetoWidget.h"
#include "private/ParetoFront.h"

#include "muk_qt_tools.h"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>

namespace gris
{
namespace muk
{
  /**
  */
  ParetoWidget::ParetoWidget(QWidget *parent)
    : QWidget(parent)
  {
    auto* gridLayout = new QGridLayout(this);
    auto* Hlayout    = new QHBoxLayout();
    auto* Vlayout    = new QVBoxLayout();
    Vlayout->addLayout(Hlayout);
    gridLayout->addLayout(Vlayout, 0, 9, 8, 4);

    auto* viewPareto = new QGraphicsView(this);
    mpParetoFront = new ParetoFront(800, 800, viewPareto);
    mpParetoFront->setObjectName("ParetoFront");
    mpParetoFront->connect(mpParetoFront, &ParetoFront::clickedPath, this, [=] (size_t ind) { emit clickedPath(ind) ; });

    viewPareto->setObjectName("Graphic View Pareto");
    viewPareto->setRenderHint(QPainter::Antialiasing);
    viewPareto->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    viewPareto->setBackgroundBrush(Qt::white);
    viewPareto->setScene(mpParetoFront);
    viewPareto->setToolTip("Shows the ParetoFront for the chosen Parameters.");
    gridLayout->addWidget(viewPareto, 0, 0, 8, 8);

    auto* comboBox = new QComboBox(this);
    comboBox->setObjectName("paretoParam1");
    comboBox->addItems(mPossibleParameterList);
    comboBox->connect(comboBox, SELECT<const QString&>::OVERLOAD_OF(&QComboBox::currentIndexChanged), this, [&] (const QString& str) 
      {
        paramChosen(true, str);
        emit parameterChosen(true, str);
      });
    Hlayout->addWidget(comboBox);

    comboBox = new QComboBox(this);
    comboBox->setObjectName("paretoParam2");
    comboBox->addItems(mPossibleParameterList);
    comboBox->connect(comboBox, SELECT<const QString&>::OVERLOAD_OF(&QComboBox::currentIndexChanged), this, [&](const QString& str) { paramChosen(false, str); emit this->parameterChosen(false, str);});
    Hlayout->addWidget(comboBox);

    auto* button = new QPushButton(this);
    button->setObjectName("paretoExitButton");
    button->setText("Close Window");
    button->setToolTip("closes the Window for the Pareto Front");
    button->connect(button, &QPushButton::clicked, [&]() { emit paretoExitClicked(); });
    Vlayout->addWidget(button);

    this->setLayout(gridLayout);
  }

  /**
  */
  ParetoWidget::~ParetoWidget()
  {
  }

  /** updates one ComboBox when a parameter is chosen in the other
  */
  void ParetoWidget::paramChosen(const bool is_param1, const QString& chosenParam) const
  {
    QComboBox* parameterBox;
    parameterBox = is_param1 ? findChild<QComboBox*>("paretoParam2") : findChild<QComboBox*>("paretoParam1");
    const QSignalBlocker blocker(parameterBox);
    auto oldText = parameterBox->currentText();
    parameterBox->clear();
    parameterBox->addItems(mPossibleParameterList);
    auto chosenIndex = parameterBox->findText(chosenParam);
    if (chosenIndex > 0)
      parameterBox->removeItem(chosenIndex);
    parameterBox->setCurrentIndex(parameterBox->findText(oldText));
  }

  /**
  */
  void ParetoWidget::setParameterList(const std::vector<std::string>& names)
  {
    mPossibleParameterList.clear();
    for (const auto& str : names)
      mPossibleParameterList.push_back(str.c_str());
    resetChoice();
  }

  /** Sets one Parameter in the ParetoFront, when both are set the paths get loaded into the scene
  */
  void ParetoWidget::setParetoParameter(bool isParam1, const QString & paramName, const std::vector<double>* parameterList, const std::vector<size_t>* parameterOrder, const std::vector<size_t>& filteredPaths)
  {
    mpParetoFront->setParameterList(isParam1, paramName, parameterList, parameterOrder, filteredPaths);
  }

  /**
  */
  void ParetoWidget::resetChoice()
  {
    setParetoParameter(true, mPossibleParameterList.front(), nullptr, nullptr, std::vector<size_t>());
    setParetoParameter(false, mPossibleParameterList.front(), nullptr, nullptr, std::vector<size_t>());
    auto* parameterBox = findChild<QComboBox*>("paretoParam1");
    const QSignalBlocker blocker(parameterBox);
    parameterBox->clear();
    parameterBox->addItems(mPossibleParameterList);
    parameterBox->setCurrentIndex(0);
    //const QSignalBlocker blocker(parameterBox);
    parameterBox = findChild<QComboBox*>("paretoParam2");
    parameterBox->clear();
    parameterBox->addItems(mPossibleParameterList);
    parameterBox->setCurrentIndex(0);
  }
}
}