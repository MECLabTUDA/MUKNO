#pragma once
#include "AlgorithmWrapper.h"
#include "muk_algorithms_api.h"

#include "MukCommon/IAbstractFactory.h"

namespace gris
{
  namespace muk
  {
    template class MUK_ALGO_API IAbstractFactory<std::string, AlgorithmWrapper>;
    typedef class MUK_ALGO_API IAbstractFactory<std::string, AlgorithmWrapper> AlgorithmFactory;

    MUK_ALGO_API AlgorithmFactory& GetAlgorithmFactory();
  }
}

#define REGISTER_ALGORITHM( type ) static bool BOOST_PP_CAT( type, __regged ) = gris::muk::GetAlgorithmFactory().register_class( gris::muk::type::s_name(), &std::make_unique<type> );