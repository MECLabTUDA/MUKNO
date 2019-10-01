#pragma once

#include <string>
#include <vector>

namespace gris
{
  namespace muk
  {
    class PlanningModel;
    class MukScene;

    /** \brief responsible for the planning with the two motion planners
    */
    class PlanningController
    {
      public:
        static void readConfigFile(const std::string& configFile, PlanningController& obj);
        static const std::string Key_Origin_Otobasis;
        static const std::string Key_A_Otobasis;
        static const std::string Key_B_Otobasis;
        static const std::string Key_Origin_SegThor;
        static const std::string Key_A_SegThor;
        static const std::string Key_B_SegThor;
        static const std::string Planner_A;
        static const std::string Planner_B;
        static const std::string Optimizer_A;
        static const std::string Optimizer_B;
        static const std::string Interpolator_A;
        static const std::string Interpolator_B;

      public:
        PlanningController();

      public:
        void setPlanningTime(double d)            { mPlanningTime = d; }
        void setPlanningModel(PlanningModel& p)   { mpPlan = &p; }
        void setScene(MukScene& p)                { mpScene = &p; }
        void setActiveKey(const std::string& key) { mCurrentKey = key;  }

      public:
        void setToBezierSplines();
        void setToCircularArcs();
        void plan();
        void save(const std::string& outputDir);

      private:
        using Parameters = std::vector<std::pair<std::string, std::string>>;

      private:
        PlanningModel* mpPlan  = nullptr;
        MukScene*      mpScene = nullptr;
        std::string    mCurrentKey;

        double        mPlanningTime;
        size_t        mNumPathsBezier;
        size_t        mNumPathsBevel;
        size_t        mNumComputedPaths;
        Parameters    mParamsBezierPlanner;
        Parameters    mParamsBezierOptimizer;
        Parameters    mParamsBezierInterpolator;
        Parameters    mParamsBevelPlanner;
        Parameters    mParamsBevelOptimizer;
        Parameters    mParamsBevelInterpolator;
    };
  }
}
