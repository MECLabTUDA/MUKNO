#pragma once

#include "MukCommon/INavigator.h"
#include "MukCommon/INavigationSupervisor.h"
#include "MukCommon/LoggerWrapper.h"

#include <QObject>

#include <memory>
#include <mutex>

namespace gris
{
namespace muk
{
  class SafePropertyReceiver;

  class SafePropertySender 
    : public QObject
    , public NavigatorProperty
  {
    Q_OBJECT

  public:
    typedef std::unique_lock<std::mutex> GuardLockType;

  public:
    SafePropertySender(QObject* parent = nullptr);
    ~SafePropertySender() {}

  public:
    void connectToReceiver(SafePropertyReceiver* pr);
    std::string getProperty(const std::string& key);
    GuardLockType guard();

    void forwardFromReceiver(const std::string& key, const std::string& value);
    void setReceiver(SafePropertyReceiver* pReceiver);

  signals:
    void propertyChanged(const std::string& key, const std::string& value);
    void propertyRequested(const std::string& key);

  private:
    std::map<std::string, std::string>    mPropMap;
    SafePropertyReceiver*                 mpReceiver;
  };

  class SafePropertyReceiver
   : public QObject
  {

    Q_OBJECT

  public:
    typedef SafePropertySender::GuardLockType GuardLockType;
    typedef io::LoggerWrapper*                PtrToLogger;

  public:
    SafePropertyReceiver(std::unique_ptr<NavigatorProperty> pProp, QObject* parent = nullptr);
    ~SafePropertyReceiver();

  public:
    SafePropertySender* getSender();
    GuardLockType       guard();
    void                moveGuardToObjectForDeletion(GuardLockType* lock = nullptr);

    void                setLogger(PtrToLogger ptr);

  public slots :
    void setProperty(const std::string& key, const std::string& value);
    void requestProperty(const std::string& key);

  private:
    std::unique_ptr<NavigatorProperty>  mpProp;
    std::unique_ptr<SafePropertySender> mpSender;
    std::mutex                          mMutex;
    GuardLockType                       mMutexGuard;

    PtrToLogger                         mpLoggerWrapper;
  };
}
}
