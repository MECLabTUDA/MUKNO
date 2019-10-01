#pragma once

#include "muk_qt_api.h"

#include <qwidget.h>
class QCustomPlot;

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_QT_API MukPlot : public QWidget
    {
      Q_OBJECT

      public:
        explicit MukPlot(QWidget* parent = nullptr);
        virtual ~MukPlot();

      public:
        void setTitle(const char* str);
        virtual void setTitleSize(unsigned int size);
        virtual void setLabelSize(unsigned int size) { mLabelSize = size; }
        virtual void setTickSize (unsigned int size) { mTickSize  = size; }
        void save();
        void toggle();
        QCustomPlot* getPlot() const { return mpPlot; }

      protected:
        QCustomPlot* mpPlot;
        unsigned int mLabelSize;
        unsigned int mTickSize;
        unsigned int mTitleSize;
        std::string  mFontName = "Helvetica";
    };

  }
}
