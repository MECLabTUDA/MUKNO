#pragma once

#include "muk_common_api.h"
#include "MukCommon/IAbstractFactory.h"

#include "CalibrationStateMachine.h"

namespace gris
{
  namespace muk
  {    
    //typedef IAbstractFactory<std::string, IMukEvaluation> EvaluationFactory;
    template class MUK_COMMON_API IAbstractFactory<std::string, CalibrationStateMachine>;
    typedef class IAbstractFactory<std::string, CalibrationStateMachine> CalibrationFactory;

    MUK_COMMON_API CalibrationFactory& GetCalibrationFactory();
  }
}

#define REGISTER_CALIBRATION_STATEMACHINE( type ) static bool BOOST_PP_CAT( type, __regged ) = gris::muk::GetCalibrationFactory().register_class( gris::muk::type::s_name(), &std::make_unique<type> );

