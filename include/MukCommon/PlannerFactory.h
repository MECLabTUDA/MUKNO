#pragma once

#include "muk_common_api.h"
#include "IAbstractFactory.h"
#include "IPathPlanner.h"

namespace gris
{
  namespace muk
  {
    template class MUK_COMMON_API IAbstractFactory<std::string, IPathPlanner>;
    typedef class IAbstractFactory<std::string, IPathPlanner> PlannerFactory;

    MUK_COMMON_API PlannerFactory& GetPlannerFactory();
  }
}

#define REGISTER_PLANNER( type ) static bool BOOST_PP_CAT( type, __regged ) = gris::muk::GetPlannerFactory().register_class( gris::muk::type::s_name(), &std::make_unique<type> );


