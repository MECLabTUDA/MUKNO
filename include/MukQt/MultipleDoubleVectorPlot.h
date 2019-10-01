#pragma once

#include "MukPlot.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_QT_API MultipleDoubleVectorPlot : public MukPlot
    {
      public:
        explicit MultipleDoubleVectorPlot(QWidget* parent = nullptr);
        virtual ~MultipleDoubleVectorPlot();

        void setNumberOfGraphs(size_t n);

        void setData(const std::vector<std::vector<double>>& data);
        void setData(size_t i, const std::vector<double>& data);
        void setGraphTitle(int i, const std::string& graphTitle);

        void adjust();

        void setXLabel(const char* str);
        void setYLabel(const char* str);

      private:
        std::vector<double> xmax;
        std::vector<double> ymax;
    };

  }
}
