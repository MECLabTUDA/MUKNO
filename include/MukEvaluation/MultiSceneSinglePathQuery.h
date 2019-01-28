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
    class MUK_EVAL_API MultiSceneSinglePathQuery : public IMukEvaluation
    {
      public:
        struct QueryData
        {
          std::vector<std::string> sceneNames;
          std::string pathName;
        };

        struct QueryOutput
        {
          QueryOutput() {}
          std::string sceneName;
          std::vector<double> distances;
          std::vector<double> curvatures;
        };
        
      public:
        MultiSceneSinglePathQuery();
        virtual ~MultiSceneSinglePathQuery();

      public:
        static  const char* s_name()       { return "MultiSceneSinglePathQuery"; };
        virtual const char* name()  const  { return s_name(); }

      public:
        virtual void        read (const std::string& xml);

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