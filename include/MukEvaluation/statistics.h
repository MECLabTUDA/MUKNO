#pragma once

#include "muk_evaluation_api.h"

#include <vector>

namespace gris
{
  namespace muk
  {
    class ICollisionDetector;
    class MukState;

    MUK_EVAL_API std::vector<double> computeDistances(ICollisionDetector& detector, const std::vector<MukState>& data, double radius);
    
    /**
    */
    struct MUK_EVAL_API BoxPlotData
    {
      double mMin;
      double mMax;
      double mUpperQuartil;
      double mLowerQuartil;
      double mMedian;

      size_t mFailures = 0;
      std::vector<double> mData;
      void compute();
    };

    /**
    */
    struct MUK_EVAL_API Statistics1D
    {
      double mMin;
      double mMax;
      double mUpperQuartil;
      double mLowerQuartil;
      double mMedian;
      double mMean;
      double mStdev;

      std::vector<double> mData;
      void compute();
    };


    MUK_EVAL_API std::vector<double> computeCurvatures(const std::vector<MukState>& data);
    MUK_EVAL_API double computeLength(const std::vector<MukState>& data);
  }
}