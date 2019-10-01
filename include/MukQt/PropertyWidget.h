#pragma once

#include "muk_qt_api.h"

#include "MukCommon/MukVector.h"

#include <qtreewidget.h>

namespace gris
{
  namespace muk
  {

    class MUK_QT_API PropertyWidget : public QTreeWidget
    {
      Q_OBJECT

      public:
        PropertyWidget(QWidget* parent = nullptr);
        ~PropertyWidget();

      signals:
        //void propertyChanged(const std::string& prop, const std::string& value);
        void propertyChanged();
        // when the activ property changes (for obstacles) this signal is send to SelectionController
        void activChanged();

      public:
        const std::string& getName()  const { return mNameCache; }
        const std::string& getValue() const { return mValueCache; }
        std::vector<std::string> getParentProperties() const;

      public:
        void addTopLevelProperty(const std::string& propName);
        void topLevelProperty(const std::string& propName);
        void addProperty(const std::string& propName);
        void property(const std::string& propName);
        void parentProperty();

        void addString(const std::string& str, const std::string& value);
        void addColor (const std::string& str, const Vec3d& color);
        void addBool  (const std::string& str, bool b);
        void addStringSelection(const std::string& str, const std::string& value, const std::vector<std::string>& possibilities);

      private:
        std::string mNameCache;
        std::string mValueCache;
        QTreeWidgetItem* mpCurrentProperty;
    };

  }
}