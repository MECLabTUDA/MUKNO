#pragma once

#include "muk_common_api.h"

#include "gstd/dynamicProperty.h"

namespace gris
{
  namespace muk
  {
    class EvaluationWindow;
    /**
    */
    class MUK_COMMON_API IMukEvaluation : public gstd::DynamicProperty
    {
      public:
        IMukEvaluation();
        virtual ~IMukEvaluation();

      public:
        virtual const char* name()  const = 0;

      public:
        void  setQuery(const std::string& text) { mQuery = text; }
        const std::string& getQuery()           { return mQuery; }

      public:
        virtual bool validateQueryInput() = 0;
        virtual void evaluate() = 0;

        virtual void saveResults(const char* filename) const = 0;
        virtual void loadResults(const char* filename) = 0;

        virtual void visualize(EvaluationWindow* pWindow) = 0;

      protected:
        std::string mQuery;
    };

  }
}