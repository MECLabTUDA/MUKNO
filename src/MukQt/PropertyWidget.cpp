#include "private/muk.pch"
#include "PropertyWidget.h"
#include "muk_qt_tools.h"

#include "MukCommon/MukException.h"

#include <qpushbutton.h>
#include <qcolordialog.h>
#include <qcheckbox.h>
#include <QLineEdit.h>
#include <QCombobox.h>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <sstream>

namespace
{
  using namespace gris;
  bool selectColor(QPushButton* button);
  QTreeWidgetItem* findWidget(QTreeWidget* pWidget, const std::string& name);
  QTreeWidgetItem* findChild(QTreeWidgetItem* pWidget, const std::string& name);
}

namespace gris
{
namespace muk
{
  /**
  */
  PropertyWidget::PropertyWidget(QWidget* parent)
    : QTreeWidget(parent)
    , mpCurrentProperty(nullptr)
  {
    setObjectName("PropertyWidget");
    QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(this->sizePolicy().hasHeightForWidth());
    setSizePolicy(sizePolicy);
    setMaximumSize(QSize(16777215, 16777215));

    auto* item = this->headerItem();
    item->setText(0, "Property");
    item->setText(1, "Value");
    int combinedWidth = columnWidth(0) + columnWidth(1);
    setColumnWidth(0, (int)(2/3.0 * combinedWidth + 0.5));

    setStyleSheet("QLineEdit { border: none }");
    setIndentation(10); // default is 20
  }

  /**
  */
  PropertyWidget::~PropertyWidget()
  {
  }

  /**
  */
  void PropertyWidget::addTopLevelProperty(const std::string& propName)
  {
    if (nullptr != findWidget(this, propName))
    {
      throw MUK_EXCEPTION("Property already exists!", propName.c_str());
    }
    mpCurrentProperty = new QTreeWidgetItem();
    mpCurrentProperty->setText(0, propName.c_str());
    addTopLevelItem(mpCurrentProperty);
  }

  /**
  */
  void PropertyWidget::topLevelProperty(const std::string& propName)
  {
    auto* pitem = findWidget(this, propName);
    if (nullptr == pitem)
    {
      throw MUK_EXCEPTION("Property already exists!", propName.c_str());
    }
    mpCurrentProperty = pitem;
  }

  /**
  */
  void PropertyWidget::addProperty(const std::string& propName)
  {
    if (mpCurrentProperty == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE("No Current Property set!");
    }
    auto* pItem = new QTreeWidgetItem();
    pItem->setText(0, propName.c_str());
    mpCurrentProperty->addChild(pItem);
    mpCurrentProperty = pItem;
  }

  /**
  */
  void PropertyWidget::property(const std::string& propName)
  {
    if (mpCurrentProperty == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE("No Current Property set!");
    }
    auto* pItem = ::findChild(mpCurrentProperty, propName);
    if (pItem == nullptr)
    {
      const std::string info = "current prop : " + std::string(mpCurrentProperty->text(0).toLocal8Bit().constData()) + std::string(" requested child: ") + propName;
      throw MUK_EXCEPTION("Property does not exist!", info.c_str());
    }
    mpCurrentProperty = pItem;
  }

  /**
  */
  void PropertyWidget::parentProperty()
  {
    mpCurrentProperty = mpCurrentProperty->parent();
  }

  /**
  */
  void PropertyWidget::addString(const std::string& str, const std::string& value)
  {
    if (mpCurrentProperty == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE("No Current Property set!");
    }
    auto* pItem = new QTreeWidgetItem();
    mpCurrentProperty->addChild(pItem);
    pItem->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
    pItem->setText(0, str.c_str());
    insertTopLevelItem(topLevelItemCount(), pItem);
    auto* pLineEdit = new QLineEdit();
    pLineEdit->setText(value.c_str());
    {
      connect(pLineEdit, &QLineEdit::editingFinished, [this, pItem, pLineEdit] ()
      {
        if (pLineEdit->isModified())
        {
          this->mNameCache = pItem->text(0).toLocal8Bit().constData();
          this->mValueCache = pLineEdit->text().toLocal8Bit().constData();
          pLineEdit->setModified(false);
          this->mpCurrentProperty = pItem->parent();
          emit propertyChanged();
        }
      });
    }
    setItemWidget(pItem, 1, pLineEdit);
  }

  /**
  */
  void PropertyWidget::addColor(const std::string& str, const Vec3d& color)
  {
    if (mpCurrentProperty == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE("No Current Property set!");
    }
    auto* pItem = new QTreeWidgetItem();
    mpCurrentProperty->addChild(pItem);
    pItem->setText(0, str.c_str());
    insertTopLevelItem(topLevelItemCount(), pItem);
    auto* button = new QPushButton("select");
    {
      Vec3d adjustedColor = 255*color;
      button->setStyleSheet((boost::format("background-color: rgb(%d,%d,%d);") % adjustedColor.x() % adjustedColor.y() % adjustedColor.z()).str().c_str());
      connect(button, &QPushButton::clicked, [button, pItem, this] ()
      {
        if (selectColor(button))
        {
          mNameCache = pItem->text(0).toLocal8Bit().constData();
          auto curveColor = button->palette().color(QPalette::Button);
          std::stringstream ss;
          ss << curveColor.red() / 255.0 << " " << curveColor.green() / 255.0 << " " << curveColor.blue() / 255.0;
          mValueCache = ss.str();
          this->mpCurrentProperty = pItem->parent();
          emit propertyChanged();
        }
      }
      );
    }
    setItemWidget(pItem, 1, button);
  }

  /**
  */
  void PropertyWidget::addBool(const std::string& str, bool b)
  {
    if (mpCurrentProperty == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE("No Current Property set!");
    }
    auto* pItem = new QTreeWidgetItem();
    mpCurrentProperty->addChild(pItem);
    pItem->setText(0, str.c_str());
    insertTopLevelItem(topLevelItemCount(), pItem);
    auto* box = new QCheckBox();
    box->setCheckable(true);
    box->setChecked(b);
    connect(box, &QCheckBox::stateChanged, [pItem, box, this] (int)
    {
      mNameCache = pItem->text(0).toLocal8Bit().constData();
      mValueCache = boost::lexical_cast<std::string>(box->isChecked());
      this->mpCurrentProperty = pItem->parent();
      if (pItem->text(0) == "Active")
        emit activChanged();
      emit propertyChanged();
    });
    this->setItemWidget(pItem, 1, box);
  }

  /**
  */
  void PropertyWidget::addStringSelection(const std::string& str, const std::string& value, const std::vector<std::string>& possibilities)
  {
    if (mpCurrentProperty == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE("No Current Property set!");
    }
    auto* pItem = new QTreeWidgetItem();
    mpCurrentProperty->addChild(pItem);
    pItem->setText(0, str.c_str());
    insertTopLevelItem(topLevelItemCount(), pItem);
    auto* pBox = new QComboBox();
    {
      for (auto& str : possibilities)
      {
        pBox->addItem(str.c_str());
      }
      pBox->setCurrentText(value.c_str());
      connect(pBox, SELECT<const QString &>::OVERLOAD_OF(&QComboBox::currentIndexChanged), [pBox, pItem, this] (const auto& qstring)
      {
        mNameCache = pItem->text(0).toLocal8Bit().constData();
        mValueCache = qstring.toLocal8Bit().constData();
        this->mpCurrentProperty = pItem->parent();
        emit propertyChanged();
      }
      );
    }
    setItemWidget(pItem, 1, pBox);
  }

  /*
  */
  std::vector<std::string> PropertyWidget::getParentProperties() const
  {
    if (mpCurrentProperty == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE("No Current Property selected!");
    }
    std::vector<std::string> result;
    QTreeWidgetItem* pItem = mpCurrentProperty;
    QTreeWidgetItem* parent = pItem->parent();
    do
    {
      parent = pItem->parent();
      result.push_back(pItem->text(0).toLocal8Bit().constData());
      pItem = parent; 
    }
    while(parent != nullptr);
    std::reverse(result.begin(), result.end());
    return result;
  }
}
}

namespace
{
  /**
  */
  bool selectColor(QPushButton* button)
  {
    auto dialog = std::make_unique<QColorDialog>();
    dialog->exec();
    if (0 == dialog->result()) // cancel-button = 0, "x"-button = 0, ok-button = 1
      return false;
    QColor qcolor = dialog->selectedColor();
    int r, g, b;
    qcolor.getRgb(&r, &g, &b);
    {
      button->setStyleSheet((boost::format("background-color: rgb(%d,%d,%d);") % r % g % b).str().c_str());        
    }
    return true;
  }

  /**
  */
  QTreeWidgetItem* findWidget(QTreeWidget* pWidget, const std::string& name)
  {
    const int N = pWidget->topLevelItemCount();
    for (int i(0); i<N; ++i)
    {
      auto* pItem = pWidget->topLevelItem(i);
      if (pItem->text(0) == name.c_str())
      {
        return pItem;
      }
    }
    return nullptr;
  }

  /**
  */
  QTreeWidgetItem* findChild(QTreeWidgetItem* pWidget, const std::string& name)
  {
    for (int i(0); i<pWidget->childCount(); ++i)
    {
      auto* pChild = pWidget->child(i);
      if (pChild->text(0) == name.c_str())
        return pChild;
    }
    return nullptr;
  }
}