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

        static std::vector<long long> milliseconds;

      public:
        PlanningController();

      public:
        void setPlanningTime(double d)            { mPlanningTime = d; }
        void setPlanningModel(PlanningModel& p)   { mpPlan = &p; }
        void setScene(MukScene& p)                { mpScene = &p; }

      public:
        void plan(const std::string& key);
        void optimize(const std::string& reference, const std::string& target);
        void save(const std::string& outputDir, const std::string& key);

      public:
        const std::vector<std::string>& getAccesses() const { return mAccesses; }

      private:
        using Parameters = std::vector<std::pair<std::string, std::string>>;

      private:
        PlanningModel* mpPlan  = nullptr;
        MukScene*      mpScene = nullptr;
        double         mPlanningTime;

        std::vector<std::string> mAccesses;
        std::string   mPlanner;
        std::string   mOptimizer;
        std::string   mInterpolator;
        Parameters    mParamsBezierPlanner;
        Parameters    mParamsBezierOptimizer1;
        Parameters    mParamsBezierOptimizer2;
        Parameters    mParamsBezierInterpolator;
    };
  }
}
