#pragma once

#include "MukEvaluation/Experiment.h"

#include "MukCommon/MukVector.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MmwhsExperiment : public Experiment
    {
      public:
        const static std::string SubSetDir1;
        const static std::string SubSetDir2;
        const static std::string TrainingDir1;
        const static std::string TrainingDir2;
        enum EnSubSet
        {
          enSubset1,
          enSubset2
        };
        enum EnPlanningPart
        {
          enGt,
          enUnet,
          enShapeReg
        };

      public:
        MmwhsExperiment();
        virtual ~MmwhsExperiment();

      public:
        virtual void initialize(const XmlNode& node) override;
        virtual void run() override;
        virtual void evaluate() override;
        virtual void finalize(XmlNode& node) override;
        virtual void print() override;

      private:
        // segmentation stuff
        void createSurfaces(EnSubSet en);
        void createCorrespondences(EnSubSet en);
        void createSSMModels(EnSubSet en);
        void createTrainingFiles(EnSubSet en);
        void createAAMModels(EnSubSet en);
        void unetToMhd(EnSubSet en);
        void unetToPolydata(EnSubSet en);
        void predictPasm(EnSubSet en);
        void combinePasm(EnSubSet en);
        // planning
        void createPolyData(EnSubSet enSet, EnPlanningPart enPlan);
        void cutSurfaces(EnSubSet enSet, EnPlanningPart enPlan);
        void createScenes(EnSubSet enSet, EnPlanningPart enPlan);
        void createPaths(EnSubSet enSet, EnPlanningPart enPlan);
        // evaluating segmentation
        void evaluateSegmentation();
        void evaluatePlanning();

        // basics
        bool skipLabel(const std::string& label) const;

      private:
        /** \brief
        */
        struct CuttingInfo
        {
          Vec3d  positionRARV; // RA = Right Atrium, RV = Right Ventricle
          Vec3d  directionRARV; // for cylinder normal
          double radiusRARV;
          Vec3d  positionRAPA; // PA = Pulmonary Artery
          Vec3d  directionRAPA;
          double radiusRAPA;
        };
        std::map<int, CuttingInfo> readCuttingInfo(const std::string& filename);

        /** \brief
        */
        struct PlanningConfiguration
        {
          PlanningConfiguration();

          std::string timeStampGt;
          std::string timeStampUnet;
          std::string timeStampSR;
          std::string accessCanal;
          std::string planner;
          std::string optimizer;
          std::string interpolator;
          double      planningTime;

          using KeyValuePair = std::pair<std::string, std::string>;
          std::vector<KeyValuePair> plannerParams;
          std::vector<KeyValuePair> optimizerParams;
          std::vector<KeyValuePair> interpolatorParams;

          gstd::DynamicProperty props;
          gstd::DynamicProperty propsPlanner;
          gstd::DynamicProperty propsOptimizer;
          gstd::DynamicProperty propsInterpolator;

          void initialize(const XmlNode& node);
          void finalize(XmlNode& node);
          void print() const;
        };

      private:
        // turn computation on/off for either dataset
        bool mBuildSubset1;
        bool mBuildSubset2;
        // segmentation
        bool mBuildSurfaces;
        bool mBuildCorrespondences;
        bool mBuildSSMs;
        bool mBuildTrainingFiles;
        bool mBuildAAMs;
        bool mUnetToMhd;
        bool mUnetToPolydata;
        bool mPredictPasm;
        bool mCombinePasm;
        // path planning
        bool mPlanOnGT;     /// use this data set
        bool mPlanOnUnet;   /// use this data set
        bool mPlanOnShape;  /// use this data set
        bool mCreatePolyData; /// activate extraction or copying of polydata
        bool mCutSurfaces;    /// activate cropping out regions from that polydata
        bool mCreateScenes;   /// use the default scenes in ./resources to create individual ones for GT / Unet / Shape
        bool mCreatePaths;
        // evaluation
        bool mEvalSegmentation;
        bool mComputeMetrics;
        bool mComputeUnetMetric;
        bool mComputePasmMetric;
        bool mWriteMetricTable;
        bool mEvalPlanning;
        bool mComputeUnetPlanning;
        bool mComputePasmPlanning;
        bool mWritePlanningTable;

        std::string mDataBasePath;
        std::vector<std::pair<std::string, int>> mLabels; /// the following two are a work in progress...
        std::vector<int> mKnownLabels;
        std::vector<int> mActiveLabels;
        std::vector<int> mSubset1;
        std::vector<int> mSubset2;

        PlanningConfiguration mPlanningConfig;

        std::string mUnetToOriginalLabelsAlgo;
        std::string mUnetToPasmInitAlgo;
        std::string mPasmAlgo;
        std::string mCombinePasmAlgo;
        std::string mLabelToPolyDataAlgo;

        std::string mUnetResultDir;       /// predictions of latest unet run converted tpo mhd lie here
        std::string mUnetResultDirNii;    /// predictions of latest unet run lie here (.nii files)
        std::string mUnetExtractedDir; /// extracted polydata from Unet lie here
        std::string mPasmOutputIndividualSubDir;    /// segmentations from individual pasms lie here
        std::string mPasmOutputDir;   /// combined segmentations from pasm lie here

        std::string mCutSurfaceConfigFile;   /// configuration file defining locations of cutting
        std::string mPlanningDataDir;    /// directory where cutted surfaces for planning are located
        std::string mPlanningDirGT;   /// subdir for GT results
        std::string mPlanningDirUnet; /// subdir for U-Net results
        std::string mPlanningDirSR;   /// subdir for Shape regularization results
        std::string mPlanningResultDir;    /// directory where scenes files and paths are located
    };
  }
}
