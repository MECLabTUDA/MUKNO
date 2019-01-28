#pragma once
#include "muk_qt_api.h"

#include <QWidget.h>

#include <memory>

namespace gris
{
  namespace muk
  {
    class SliceWidget;
    class VtkWindow;

    /**
    */
    class MUK_QT_API MedicalMultiViewWidget : public QWidget
    {
      Q_OBJECT
        
      public:
        explicit MedicalMultiViewWidget(QWidget* parent = nullptr);
        virtual ~MedicalMultiViewWidget();

      public:
        VtkWindow*        get3DWindow();
        const VtkWindow*  get3DWindow() const;

        SliceWidget*      getAxialSliceWidget();
        SliceWidget*      getSagittalSliceWidget();
        SliceWidget*      getCoronalSliceWidget();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
