#pragma once

#include "muk_qt_api.h"
#include "SphereInteraction.h"

#include "MukCommon/MukState.h"
#include "MukCommon/MukProblemDefinition.h"

#include <qwidget.h>
#include <QtWidgets/QTreeWidget>

QT_BEGIN_NAMESPACE
class QToolButton;
QT_END_NAMESPACE

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_QT_API ProblemDefinitionWidget : public QWidget
    {
      Q_OBJECT

      private:
        using EnRegionType_t = MukProblemDefinition::EnRegionType;

      public:
        explicit ProblemDefinitionWidget(QWidget *parent = 0);
        ~ProblemDefinitionWidget();

      signals:
        /*void updateXClicked(double val);
        void updateYClicked(double val);
        void updateZClicked(double val);*/
        void addStartRegionClicked();
        void addGoalRegionClicked();
        //void addWaypointClicked();
        void deleteRegionClicked();
        void reverseClicked();
        //void undoClicked();

      public:
        void setSelection(EnRegionType_t, int);
        EnRegionType_t getSelectedType()  const;
        int            getSelectedIndex() const;

        void setState    (const MukState& state);
        void setPosition (const Vec3d& p);
        void setDirection(const Vec3d& q);
                
        void clearState();
        void setPrecision(int numDecimals)    { mNumDecimals = numDecimals; }

      public:
        QToolButton* mpToolButtonAddStart;
        QToolButton* mpToolButtonAddGoal;
        QToolButton* mpToolButtonAddWaypoint;
        QToolButton* mpToolButtonDeleteRegion;
        QToolButton* mpToolButtonUndo;
        QToolButton* mpToolButtonReverseWaypoints;
        QToolButton* mpToolButtonShowOptimization;
        QToolButton* mpToolButtonOptimizeWaypoints;

        QTreeWidget* mpTreeWidgetWaypointVis;

        /*QLineEdit* mpLineEditX;
        QLineEdit* mpLineEditY;
        QLineEdit* mpLineEditZ;*/

        SphereInteraction* mpDirectionWidget;

      private:
        int mNumDecimals;
    };
  }
}
