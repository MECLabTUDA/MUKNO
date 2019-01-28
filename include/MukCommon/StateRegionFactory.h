#pragma once

#include "IAbstractFactory.h"
#include "IStateRegion.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    template class MUK_COMMON_API IAbstractFactory<std::string, IStateRegion>;
    typedef class IAbstractFactory<std::string, IStateRegion> StateRegionFactory;

    MUK_COMMON_API StateRegionFactory& GetStateRegionFactory();
  }
}

#define REGISTER_REGION( type ) static bool BOOST_PP_CAT( type, __regged ) = gris::muk::GetStateRegionFactory().register_class( gris::muk::type::s_name(), &std::make_unique<type> );
