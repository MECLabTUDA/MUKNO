#pragma once

#include "muk_common_api.h"
#include "IAbstractFactory.h"
#include "INavigator.h"

namespace gris
{
  namespace muk
  {    
    //typedef IAbstractFactory<std::string, IMukEvaluation> EvaluationFactory;
    template class MUK_COMMON_API IAbstractFactory<std::string, INavigator>;
    typedef class IAbstractFactory<std::string, INavigator> NavigatorFactory;

    MUK_COMMON_API NavigatorFactory& GetNavigatorFactory();
  }
}

#define REGISTER_NAVIGATOR( type ) static bool BOOST_PP_CAT( type, __regged ) = gris::muk::GetNavigatorFactory().register_class( gris::muk::type::s_name(), &std::make_unique<type> );

