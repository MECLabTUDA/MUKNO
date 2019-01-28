#pragma once

#include "BaseModel.h"

#include "MukCommon/IMukEvaluation.h"

#include "gstd/dynamicProperty.h"

#include <memory>

namespace gris
{
  namespace muk
  {
    class TabEvaluation;
    class EvaluationWindow;

    /**
    */
    class EvaluationModel : public BaseModel
    {
      public:
        EvaluationModel();
        ~EvaluationModel();

      public:
        void setupConnections(TabEvaluation* tab);
        void setEvaluationWindow(EvaluationWindow* pObj) { mpEvalWindow = pObj; }

      public:
        void                setQuery(const std::string& text);
        const std::string&  getQuery() const;
        const IMukEvaluation* getEval() const                               { return mEval.get(); }
        void                  setEval(std::unique_ptr<IMukEvaluation> eval) { mEval = std::move(eval); }

      public:
        void evaluate();
        void saveQuery(const char* filename) const;
        void setEvaluation(const char* type);

        void saveResults(const char* filename) const;
        void loadResults(const char* filename);
        
      private:
        EvaluationWindow*               mpEvalWindow;
        std::unique_ptr<IMukEvaluation> mEval;
    };
  }
}