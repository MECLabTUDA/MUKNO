#pragma once

#include "muk_common_api.h"
#include "IAbstractFactory.h"
#include "IPathOptimizer.h"

namespace gris
{
  namespace muk
  { 
    template class MUK_COMMON_API IAbstractFactory<std::string, IPathOptimizer>;
    typedef class IAbstractFactory<std::string, IPathOptimizer> OptimizerFactory;

    MUK_COMMON_API OptimizerFactory& GetOptimizerFactory();
  }
}

#define REGISTER_OPTIMIZER( type ) static bool BOOST_PP_CAT( type, __regged ) = gris::muk::GetOptimizerFactory().register_class( gris::muk::type::s_name(), &std::make_unique<type> );


