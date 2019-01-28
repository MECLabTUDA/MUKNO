#pragma once

#include "muk_common_api.h"
#include "IAbstractFactory.h"
#include "IMukEvaluation.h"

namespace gris
{
  namespace muk
  {    
    //typedef IAbstractFactory<std::string, IMukEvaluation> EvaluationFactory;
    template class MUK_COMMON_API IAbstractFactory<std::string, IMukEvaluation>;
    typedef class IAbstractFactory<std::string, IMukEvaluation> EvaluationFactory;
    
    MUK_COMMON_API EvaluationFactory& GetEvaluationFactory();
  }
}

#define REGISTER_EVALUATION( type ) static bool BOOST_PP_CAT( type, __regged ) = gris::muk::GetEvaluationFactory().register_class( gris::muk::type::s_name(), &std::make_unique<type> );