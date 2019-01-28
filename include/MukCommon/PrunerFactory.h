#pragma once

#include "muk_common_api.h"
#include "IAbstractFactory.h"
#include "IPathPruner.h"

namespace gris
{
  namespace muk
  { 
    template class MUK_COMMON_API IAbstractFactory<std::string, IPathPruner>;
    typedef class IAbstractFactory<std::string, IPathPruner> PrunerFactory;

    MUK_COMMON_API PrunerFactory& GetPrunerFactory();
  }
}

#define REGISTER_PRUNER( type ) static bool BOOST_PP_CAT( type, __regged ) = gris::muk::GetPrunerFactory().register_class( gris::muk::type::s_name(), &std::make_unique<type> );


