#pragma once

#include "muk_qt_api.h"

#include <QWidget>

namespace gris
{
  namespace muk
  {
    class ParetoFront;

    /** \brief
    */
    class MUK_QT_API ParetoWidget : public QWidget
    {
      Q_OBJECT
        
      signals:
        void clickedPath(size_t);
        void paretoExitClicked();
        void parameterChosen(bool isParam1, const QString& paramName);

      public:
        ParetoWidget(QWidget *parent = nullptr);
        ~ParetoWidget();

      public:
        void setParameterList(const std::vector<std::string>& names);
        void setParetoParameter(bool isParam1, const QString & paramName, const std::vector<double>* parameterList, const std::vector<size_t>* parameterOrder, const std::vector<size_t>& filteredPaths);
        void resetChoice();

      private:
        void paramChosen(const bool is_param1, const QString& chosenParam) const;

      private:
        ParetoFront* mpParetoFront = nullptr;
        QStringList  mPossibleParameterList;
    };
  }
}
