#pragma once

#include "muk_common_api.h"
#include "IAbstractFactory.h"
#include "IInterpolator.h"

namespace gris
{
  namespace muk
  {    
    template class MUK_COMMON_API IAbstractFactory<std::string, IInterpolator>;
    typedef class IAbstractFactory<std::string, IInterpolator> InterpolatorFactory;

    MUK_COMMON_API InterpolatorFactory& GetInterpolatorFactory();
  }
}

#define REGISTER_INTERPOLATOR( type ) static bool BOOST_PP_CAT( type, __regged ) = gris::muk::GetInterpolatorFactory().register_class( gris::muk::type::s_name(), &std::make_unique<type> );
