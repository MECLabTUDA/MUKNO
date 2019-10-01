#pragma once

#include "muk_qt_api.h"

#include "MukPathPlanning/QuickPlanningConfigurator.h"

#include <QDialog.h>
#include <qspinbox.h>
#include <QComboBox.h>
#include <QLineEdit.h>

namespace gris
{
  namespace muk
  {
    /** \brief A widget providing several input options with default configurations for planning
    */
    class MUK_QT_API QuickPlanningConfigurationWindow : public QDialog
    {
      Q_OBJECT

      public:
        QuickPlanningConfigurationWindow();

      public:
        void addConfig(const QuickPlanningConfigurator& config);
        void reloadConfigs();

        std::string getPlanner() const;
        std::string getInterpolator() const;
        std::string getPruner() const;
        size_t      getSphereResolution() const;
        size_t      getNewPaths() const;
        double      getCalcTime() const;
        double      getStepSize() const;

      private:
        void plannerSelected(const QString& str);
        
      private:
        std::vector<QuickPlanningConfigurator> mConfigs;
        QComboBox* mPlanner;
        QLineEdit* mPruner;
        QLineEdit* mInterpolator;
        QSpinBox*  mSphereResolution;
        QSpinBox*  mNewPaths;
        QDoubleSpinBox* mCalcTime;
        QDoubleSpinBox* mStepSize;
    };
  }
}
