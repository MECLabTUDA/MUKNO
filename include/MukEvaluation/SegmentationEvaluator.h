#pragma once
#include "muk_evaluation_api.h"

#include "MukImaging/MukImage.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    struct MUK_EVAL_API SegmentationResultMetrics
    {
      double jaccard;
      double dice;
      double falsePositiveError;
      double falseNegativeError;
      double sensitivity;
      double specificity;
      double hausdorffDist;
    };

    /**
    */
    class MUK_EVAL_API SegmentationEvaluator
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
