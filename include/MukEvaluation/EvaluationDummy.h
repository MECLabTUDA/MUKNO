#pragma once

#include "muk_evaluation_api.h"

#include "MukCommon/IMukEvaluation.h"

#include "MukQt/EvaluationWindow.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_EVAL_API EvaluationDummy : public IMukEvaluation
    {
      public:
        EvaluationDummy() {}
        virtual ~EvaluationDummy() {}

      public:
        static const char* s_name()      { return "EvaluationDummy"; }
        virtual const char* name() const { return s_name(); }

        virtual void saveResults(const char* filename) const {}
        virtual void loadResults(const char* filename) {}

        virtual void visualize(EvaluationWindow* pWindow) {}
        
      public:
        virtual bool validateQueryInput() { return true; }
        virtual void evaluate();
    };
  }
}
