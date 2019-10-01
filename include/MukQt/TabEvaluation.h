#pragma once

#include "muk_qt_api.h"

#include <qwidget.h>
#include <qmainwindow.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_QT_API TabEvaluation : public QWidget
    {
      Q_OBJECT

      public:
        explicit TabEvaluation(QWidget *parent = 0);
        ~TabEvaluation();

      signals:          
        void evaluate();
        void loadXml (const char*);
        void saveXml (const char*);
        void setEvaluation(const char*);
        void loadResults (const char*);
        void saveResults (const char*);

      private slots:
        void buttonLoad();
        void buttonSave();
        void catchTextChanged();

        void buttonLoadResults();
        void buttonSaveResults();

      public:
        void retranslateUi(QMainWindow *MainWindow);
        void setupConnections();

      private:
    };
  }
}