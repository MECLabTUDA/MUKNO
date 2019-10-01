#include "private/muk.pch"
#include "TabEvaluation.h"
#include "muk_qt_tools.h"

#include "MukCommon/EvaluationFactory.h"

#include <qlayout.h>
#include <qlabel.h>
#include <qcombobox.h>
#include <qpushbutton.h>
#include <qapplication.h>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/qpushbutton.h>
#include <qaction.h>

#include <QFileDialog>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <iostream>

namespace
{
  namespace fs = boost::filesystem;  
}

namespace gris
{
namespace muk
{
  /**

  Label + Combobox: Type Combobox
  ^
  |
  v
  Buttons: Load Save Eval
  TextEdit: (for xml string)
  */
  TabEvaluation::TabEvaluation(QWidget *parent)
    : QWidget(parent)
  {
    this->setObjectName("TabEvaluation");
    {
      QSizePolicy sizePolicy;
      sizePolicy.setHeightForWidth(this->sizePolicy().hasHeightForWidth());
      this->setSizePolicy(sizePolicy);
      this->setMaximumSize(QSize(16777215, 16777215));
    }

    // major layout
    auto mainLayout = std::make_unique<QVBoxLayout>(this);
    mainLayout->setObjectName(QStringLiteral("mainLayout"));

    auto topLayout = new QGridLayout();
    topLayout->setObjectName(QStringLiteral("topLayout"));
    {
      auto label = new QLabel(this);
      auto comboBox = new QComboBox(this);
      {
        label->setObjectName(QStringLiteral("labelEvaluation"));
        comboBox->setObjectName(QStringLiteral("comboBoxEvaluation"));
        {
          QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
          sizePolicy.setHorizontalStretch(0);
          sizePolicy.setVerticalStretch(0);
          sizePolicy.setHeightForWidth(comboBox->sizePolicy().hasHeightForWidth());
          comboBox->setSizePolicy(sizePolicy);
          const auto& factory = GetEvaluationFactory();
          std::vector<std::string> keys;
          factory.getKeys(keys);
          for(const auto& key: keys)
            comboBox->addItem(key.c_str());
        }
        topLayout->addWidget(label, 0, 0, 1, 1);
        topLayout->addWidget(comboBox, 0, 1, 1, 1);
      }
    }
    mainLayout->addLayout(topLayout);
    mainLayout->addItem( new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Preferred) );

    auto nextLayout = new QGridLayout();
    {
      auto xmlTextField = new QTextEdit(this);
      xmlTextField->setObjectName("xmlTextField");
      xmlTextField->setAcceptRichText(true);
      xmlTextField->setTabStopWidth(4);    
      nextLayout->addWidget(xmlTextField, 0, 0, 1, 3);
      auto button = new QPushButton(this);
      button->setObjectName("xmlLoadButton");
      nextLayout->addWidget(button, 1, 0, 1, 1);
      button = new QPushButton(this);
      button->setObjectName("xmlSaveButton");
      nextLayout->addWidget(button, 1, 1, 1, 1);
      button = new QPushButton(this);
      button->setObjectName("xmlEvalButton");
      nextLayout->addWidget(button, 1, 2, 1, 1);
      mainLayout->addLayout(nextLayout);
    }    
    mainLayout->addItem( new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Preferred) );
    nextLayout = new QGridLayout();
    {
      auto button = new QPushButton(this);
      button->setObjectName("loadResults");
      nextLayout->addWidget(button, 0, 0, 1, 1);
      button = new QPushButton(this);
      button->setObjectName("saveResults");
      nextLayout->addWidget(button, 1, 0, 1, 1);
      mainLayout->addLayout(nextLayout);
    }
    mainLayout->addItem( new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Preferred) );

    this->setLayout(mainLayout.release());

    setupConnections();
  }

  /**
  */
  TabEvaluation::~TabEvaluation()
  {
  }
  
  /**
  */
  void TabEvaluation::setupConnections()
  {    
    //connect(this->findChild<QTextEdit*>("xmlTextField"), &QTextEdit::textChanged, this, catchTextChanged);        
    connect(this->findChild<QPushButton*>("xmlLoadButton"), &QPushButton::clicked, this, &TabEvaluation::buttonLoad);
    connect(this->findChild<QPushButton*>("xmlSaveButton"), &QPushButton::clicked, this, &TabEvaluation::buttonSave);
    connect(this->findChild<QPushButton*>("xmlEvalButton"), &QPushButton::clicked, this, [&] () { emit loadXml(this->findChild<QTextEdit*>("xmlTextField")->toPlainText().toLocal8Bit().constData()); emit evaluate(); });
    connect(this->findChild<QComboBox*>("comboBoxEvaluation"),
      SELECT<int>::OVERLOAD_OF(&QComboBox::currentIndexChanged),
      this, 
      [&] (int idx) { emit setEvaluation(this->findChild<QComboBox*>("comboBoxEvaluation")->currentText().toLocal8Bit().constData()); });

    connect(this->findChild<QPushButton*>("saveResults"), &QPushButton::clicked, this, &TabEvaluation::buttonSaveResults);
    connect(this->findChild<QPushButton*>("loadResults"), &QPushButton::clicked, this, &TabEvaluation::buttonLoadResults);    
  }

  /**
  */
  void TabEvaluation::buttonLoad()
  {
    QString qFile = QFileDialog::getOpenFileName(this, tr("Open File"), QDir::currentPath(), tr("Xml Files (*.xml)"));
    if (qFile.isEmpty())
      return;
    std::ifstream ifs(qFile.toLocal8Bit().constData());
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    this->findChild<QTextEdit*>("xmlTextField")->setText(buffer.str().c_str());
    emit loadXml(this->findChild<QTextEdit*>("xmlTextField")->toPlainText().toLocal8Bit().constData());
  }

  /**
  */
  void TabEvaluation::buttonSave()
  {
    QString qFile = QFileDialog::getSaveFileName(this, tr("Save File"), QDir::currentPath());
    if (qFile.isEmpty())
      return;    
    emit saveXml(qFile.toLocal8Bit().constData());
  }

  /**
  */
  void TabEvaluation::catchTextChanged()
  {
    emit loadXml(this->findChild<QTextEdit*>("xmlTextField")->toPlainText().toLocal8Bit().constData());
  }

  /**
  */
  void TabEvaluation::buttonLoadResults()
  {
    QString qFile = QFileDialog::getOpenFileName(this, tr("Open File"), QDir::currentPath(), tr("Text Files (*.txt)"));
    if (qFile.isEmpty())
      return;
    emit loadResults(qFile.toLocal8Bit().constData());
  }

  /**
  */
  void TabEvaluation::buttonSaveResults()
  {

    QString qFile = QFileDialog::getSaveFileName(this, tr("Save File"), QDir::currentPath(), tr("Text Files (*.txt)"));
    if (qFile.isEmpty())
      return;    
    const std::string fn = (boost::format("%s.txt") % qFile.toLocal8Bit().constData()).str();
    //emit saveResults(fn.c_str());
    emit saveResults(qFile.toLocal8Bit().constData());
  }

  /**
  */
  void TabEvaluation::retranslateUi(QMainWindow *MainWindow)
  {
    this->findChild<QLabel*>("labelEvaluation")->setText(QApplication::translate("MainWindow", "Evaluation", 0));
    this->findChild<QPushButton*>("xmlLoadButton")->setText(QApplication::translate("MainWindow", "Load", 0));
    this->findChild<QPushButton*>("xmlSaveButton")->setText(QApplication::translate("MainWindow", "Save", 0));
    this->findChild<QPushButton*>("xmlEvalButton")->setText(QApplication::translate("MainWindow", "Evaluate", 0));

    this->findChild<QPushButton*>("saveResults")->setText(QApplication::translate("MainWindow", "Save Results", 0));
    this->findChild<QPushButton*>("loadResults")->setText(QApplication::translate("MainWindow", "Load Results", 0));
  }
}
}

