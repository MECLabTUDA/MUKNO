#pragma once
#include "muk_qt_api.h"
#include "ProblemDefinitionWidget.h"

#include <QWidget>

QT_BEGIN_NAMESPACE
class QButtonGroup;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QMainWindow;
class QPushButton;
class QRadioButton;
class QScrollArea;
class QSpacerItem;
class QSpinBox;
QT_END_NAMESPACE

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_QT_API TabPlanning : public QWidget
    {
      Q_OBJECT

      public:
        explicit TabPlanning(QWidget *parent = 0);
        ~TabPlanning();

      signals:
        // path collections
        void pathCollectionChanged(const char* name);
        void pathIdxChanged(int idx);

        void addPathCollectionClicked(const char* name);
        void deletePathCollectionClicked(const char* name);        
        void clearPathCollectionClicked(const std::string& name);        

        // planning
        void createMukPathsClicked(const std::string& pathCollection, size_t numNewPaths);
        void createMukPathsClicked(const std::string& pathCollection, double timeAvailable);
        void updateMukPathClicked(const std::string& pathCollection, int pathIdx);
        void copyMukPathClicked(const std::string& pathCollection, int pathIdx);
        void addReplanningClicked(const std::string& pathCollection, int pathIdx, int stateIdx);
        void updateProblemDefinitionClicked(const std::string& pathCollection);
        
        void changeSamplingClicked();
        void updatePlannerClicked();
        void updatePrunerClicked();
        void updateInterpolatorClicked();

        void plannerChanged(const char*);
        void prunerChanged (const char*);
        void interpolatorChanged(const char*);

      public slots:
        void insertPathCollection(const std::string& name);

      private:
        // these private functions react to qt signal void clicked(bool==false)
        void addPathCollection(QComboBox* box);
        void deletePathCollection(bool=false);
        void clearPathCollection(bool=false);
        void createMukPathsNumber(QComboBox* box, QSpinBox* spin);
        void createMukPathsTime  (QComboBox* box, QDoubleSpinBox* spin);
        void updateMukPath(bool=false);
        void copyMukPath();
        void updateProblemDefinition(bool=false);

      public:
        void retranslateUi(QMainWindow *MainWindow);
        void setActivePathIdx(int value);
        void setStateIdx(int value);

      public:
        void setPathCollection (const std::string& name);
        void setPlanners       (const std::vector<std::string>& keys);
        void setPruners        (const std::vector<std::string>& keys);
        void setInterpolators  (const std::vector<std::string>& keys);
        void setPathCollections(const std::vector<std::string>& keys, int current = 0);
        void setPlanner        (const std::string& key);
        void setPruner         (const std::string& key);
        void setInterpolator   (const std::string& key);

      public:
        int  getSamlingMethod() const;
        void setSamplingSelection(int i);
        void setSamplingText(const std::string& str1, const std::string& str2);
        
      public:
        QRadioButton* mpSampling1;
        QRadioButton* mpSampling2;
        QGroupBox*    mpSamplingBox;
        QButtonGroup* mpSamplingGroup;

        QComboBox*    mpPathSelection;
        QSpinBox*     mpPathIdxSpinBox;
        QSpinBox*     mpStateIdxSpinBox;
        QPushButton*  mpPushButtonUpdateAll;
        QPushButton*  mpPushButtonCopy;

        QComboBox*    comboBoxPlanner;
        QPushButton*  pushButtonPlanner;
        QComboBox*    comboBoxInterpolator;
        QPushButton*  pushButtonInterpolator;
        QComboBox*    comboBoxPruner;
        QPushButton*  pushButtonPruner;
        
        ProblemDefinitionWidget* mpProbDefWidget;
    };
  }
}