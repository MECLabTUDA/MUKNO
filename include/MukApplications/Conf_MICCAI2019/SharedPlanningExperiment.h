#pragma once
#include "MukEvaluation/Experiment.h"

#include <vector>

namespace gris
{
  namespace muk
  {
    class MukScene;
    class PlanningController;

    /**
    */
    class SharedPlanningExperiment : public Experiment
    {
      protected:
        using ObstacleDistanceMap = std::map<std::string, double>;
        /** \brief Collects values from all scenes for a specific access path
        */
        struct AccessPathEvaluation
        {
          std::vector<size_t> nPathsTotalRRT;
          std::vector<size_t> nPathsTotalSCO;
          std::vector<double> minDistsRRT;
          std::vector<double> minDistsSCO;
          
          std::vector<ObstacleDistanceMap> minDistsIndividualRRT;
          std::vector<ObstacleDistanceMap> minDistsIndividualSCO;
        };
        using Evaluations = std::map<std::string, AccessPathEvaluation>;

      protected:
        void evaluateScene(const std::string& sceneDir, Evaluations& evalParams);
        void evaluateTotal(const std::string& dir, Evaluations& evalParams);
        void runScene(const std::string& sceneFilename, const std::string& id);
        void peakScene(const std::string& sceneFilename, MukScene& scene);
        std::string rrtKey(const std::string& prefix) const;
        std::string scoKey(const std::string& prefix) const;

      protected:
        std::unique_ptr<PlanningController> mpPlanning;

        std::string mInterpolator;

        std::string mPlannerConfigFile;
        std::string mLocalBasePath;       // base path for data of scenes
        std::vector<std::string> mAccessPaths;
        double      mPlanningTime;
    };
  }
}