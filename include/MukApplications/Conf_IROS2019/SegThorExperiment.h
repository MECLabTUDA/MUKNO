#pragma once
#include "MukEvaluation/Experiment.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class SegThorExperiment : public Experiment
    {
    public:
      SegThorExperiment();

    public:
      virtual void run()      override;
      virtual void evaluate() override;

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
      // actual planning
      std::string mPlannerConfigFile;
      double      mPlanningTime;
    };
  }
}