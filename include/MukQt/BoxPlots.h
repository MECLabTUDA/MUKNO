#pragma once

#include "MukPlot.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_QT_API BoxPlots : public MukPlot
    {
      public:
        explicit BoxPlots(QWidget* parent = nullptr);
        virtual ~BoxPlots();

      public:
        void clear();

        void addBox(const std::vector<double>& data);

        void adjust();
        void setSampleName(int index, const char* str);
        void setSampleRotation(double val);
        void setYLabel(const char* str);
        
      private:
        /*struct Set
        {
          double min;
          double max;
          double median;
          double lowerq;
          double upperq;
        };
        std::vector<Set> mSets;*/
    };

  }
}
