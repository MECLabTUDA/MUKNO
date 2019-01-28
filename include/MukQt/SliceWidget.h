#pragma once
#include "muk_qt_api.h"

#include <vtkSmartPointer.h>
class QVTKWidget;
class QLabel;

#include <QWidget>
#include <QLayout>
#include <QPushButton>

#include <memory>

namespace gris
{
  namespace muk
  {
    class SliceRenderGroup;

    /**
    */
    class MUK_QT_API SliceWidget : public QWidget
    {
      public:
        SliceWidget(QWidget* parent = nullptr);
        virtual ~SliceWidget();

      public:
        QGridLayout*            getLayout()             { return mpLayout; }
        SliceRenderGroup&       getRenderGroup()        { return *mpSliceRenderGroup; }
        const SliceRenderGroup& getRenderGroup() const  { return *mpSliceRenderGroup; }
        QPushButton*            getFullScreenButton()   { return mpToggleFullScreenButton; } 
        QLayout*    getLayout() const  { return mpLayout; };
        QVTKWidget* getWidget() const  { return mpWidget; };

      public:        
        void hide(bool b);
        void setSliceNumber(int current, int max);
        void setValue(int x, int y, int z, double v);

      private:
        QVTKWidget*   mpWidget;
        QPushButton*  mpToggleFullScreenButton;
        QGridLayout*  mpLayout;
        QLabel*       mpSliceNumberLabel;
        std::unique_ptr<SliceRenderGroup> mpSliceRenderGroup;
    };
  }
}