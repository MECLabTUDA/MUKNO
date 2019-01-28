#pragma once

#include "MukImaging/MukImage.h"


namespace gris
{
  namespace muk
  {
    /**
    */
    struct SegmentationResultMetrics
    {
      double jaccard;
      double dice;
      double falsePositiveError;
      double falseNegativeError;
      double sensitivity;
      double specificity;
    };

    /**
    */
    class SegmentationEvaluator
    {
      public:
        SegmentationEvaluator();

      public:
        void setGroundTruth(const ImageInt3D& img)  { mpGroundTruth = &img; }
        void setSegmentation(const ImageInt3D& img) { mpSegmentation = &img; }
        void setLabel(unsigned int label) { mLabel = label; }

      public:
        void update();
        const SegmentationResultMetrics& getMetrics() const { return mCurrentResult;  }

      private:
        const ImageInt3D* mpGroundTruth;
        const ImageInt3D* mpSegmentation;
        unsigned int mLabel;
        SegmentationResultMetrics mCurrentResult;
    };
  }
}
