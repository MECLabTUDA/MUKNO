#pragma once

#include "muk_evaluation_api.h"

#include "MukCommon/IMukEvaluation.h"

namespace gris
{
  namespace muk
  {
    class MukScene;

    /**
    */
    class MUK_EVAL_API PathPlanningComparisonQuery : public IMukEvaluation
    {
      public:
        struct QueryData
        {
          std::vector<std::string> sceneNames;
          std::vector<std::string> inactiveObstacles;
          std::string pathNameFirst;
          std::string pathNameSecond;
        };

        struct QueryOutput
        {
          QueryOutput() {}
          std::string sceneName;
          std::string pathNameFirst;
          std::string pathNameSecond;
          double maxDistance;
          double maxCurvature;
          std::vector<double> distancesFirst;
          std::vector<double> curvaturesFirst;
          std::vector<double> distancesSecond;
          std::vector<double> curvaturesSecond;
        };

      public:
        PathPlanningComparisonQuery();
        virtual ~PathPlanningComparisonQuery();

      public:
        static  const char* s_name()       { return "PathPlanningComparisonQuery"; };
        virtual const char* name()  const  { return s_name(); }

      public:
        virtual void read (const std::string& xml);

        virtual void saveResults(const char* filename) const;
        virtual void loadResults(const char* filename);

        virtual void visualize(EvaluationWindow* pWindow);

      public:
        virtual bool validateQueryInput();
        virtual void evaluate();
        virtual const std::vector<QueryOutput>* getOutput() const;

      private:
        void readSingleScene(MukScene* pScene);

      private:
        QueryData mInput;
        std::vector<QueryOutput> mOutput;
    };

  }
}