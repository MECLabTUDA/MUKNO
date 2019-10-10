#pragma once

#include <QObject>
#include <string>
#include "MukCommon/Typedefs.h"
#include "MukCommon/IInterpolator.h"
#include "MukCommon/MukPath.h"
#include "MukCommon/MukState.h"
#include "MukCommon/INavigator.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukVector.h"
#include "MukCommon/INavigator.h"
#include "MukCommon/MukTransform.h"

Q_DECLARE_METATYPE(std::string)
Q_DECLARE_METATYPE(std::exception)
Q_DECLARE_METATYPE(gris::muk::IInterpolator::Pointer)
Q_DECLARE_METATYPE(gris::muk::MukPath)
Q_DECLARE_METATYPE(gris::muk::MukState)
Q_DECLARE_METATYPE(gris::muk::MukException)
Q_DECLARE_METATYPE(gris::Vec3d)
Q_DECLARE_METATYPE(gris::muk::INavigator::NavigatorFeature)
Q_DECLARE_METATYPE(gris::muk::MukTransform)
Q_DECLARE_METATYPE(gris::muk::StringVector)
Q_DECLARE_METATYPE(gris::muk::enNavigatorState)
Q_DECLARE_METATYPE(gris::muk::enConsumerState)

void registerMetaTypes()
{
  // register required Types as QtMetaType
  // needed for Queued Connections
  qRegisterMetaType<std::string>();
  qRegisterMetaType<std::exception>();
  qRegisterMetaType<gris::muk::MukState>();
  qRegisterMetaType<gris::muk::MukPath>();
  qRegisterMetaType<gris::muk::IInterpolator::Pointer>();
  qRegisterMetaType<gris::muk::MukException>();
  qRegisterMetaType<gris::Vec3d>();
  qRegisterMetaType<gris::muk::INavigator::NavigatorFeature>();
  qRegisterMetaType<gris::muk::MukTransform>();
  qRegisterMetaType<gris::muk::StringVector>();
  qRegisterMetaType<gris::muk::enNavigatorState>();
  qRegisterMetaType<gris::muk::enConsumerState>();
}
