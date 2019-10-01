#pragma once
#include "SharedPlanningExperiment.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class SegThorExperiment : public SharedPlanningExperiment
    {
      public:
        SegThorExperiment();

      public:
        virtual void run()      override;
        virtual void evaluate() override;
        virtual void initialize(const XmlNode& node) override;
        virtual void print()    override;

        // run
      private:
        void extractSegThor();
        void postProcessSegThor();
        void copyBaseScene();

      private:
        bool mExtractDataSet;
        bool mPostProcessDataSet;
        bool mCopyBaseScene;

        std::string mDataDirSegThor;
        // extracting data
        std::string mAlgExtractLabels;
        double      mAortaToHeartDistanceThreshold;
        double      mAortaMeanRadius;
    };
  }
}