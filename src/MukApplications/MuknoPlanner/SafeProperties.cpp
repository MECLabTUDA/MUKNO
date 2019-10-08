#include "private/muk.pch"
#include "SafeProperties.h"

#include <QThread>

namespace gris
{
namespace muk
{
  SafePropertySender::SafePropertySender(QObject* p)
    : QObject(p)
    , NavigatorProperty()
  {
  }

  void SafePropertySender::connectToReceiver(SafePropertyReceiver * pr)
  {
    setReceiver(pr);
    connect(this, &SafePropertySender::propertyChanged, pr, &SafePropertyReceiver::setProperty);
    connect(this, &SafePropertySender::propertyRequested, pr, &SafePropertyReceiver::requestProperty, Qt::BlockingQueuedConnection);
  }

  std::string SafePropertySender::getProperty(const std::string & key)
  {
    emit propertyRequested(key);
    return mPropMap[key];
  }

  SafePropertySender::GuardLockType SafePropertySender::guard()
  {
    return mpReceiver->guard();
  }

  void SafePropertySender::forwardFromReceiver(const std::string & key, const std::string & value)
  {
    mPropMap[key] = value;
  }

  void SafePropertySender::setReceiver(SafePropertyReceiver * pReceiver)
  {
    if (pReceiver->thread() == thread())
      throw MUK_EXCEPTION_SIMPLE("SafeProperty Sender and Receiver are on the same Thread.")
    mpReceiver = pReceiver;
  }

  // takes possession of DynamicProperty
  SafePropertyReceiver::SafePropertyReceiver(std::unique_ptr<NavigatorProperty> pProp, QObject * parent)
    : QObject(parent)
    , mMutex()
    , mMutexGuard(mMutex)
  {
    mpProp.swap(pProp);
    mMutexGuard.unlock();
  }

  SafePropertyReceiver::~SafePropertyReceiver()
  {
    if (mpLoggerWrapper && (!mMutexGuard.owns_lock() && !mMutexGuard.try_lock()))
       *mpLoggerWrapper << "Trying to delete SafePropertyReceiver while the Mutex is locked." << std::endl 
        << "Try moving the lock to the Object with SafePropertyReceiver::moveGuardToObjectForDeletion" << std::endl;
  }

  SafePropertySender* SafePropertyReceiver::getSender()
  {
    if (!mpSender)
    {
      mpSender = std::make_unique<SafePropertySender>();
      SafePropertySender* pSender = mpSender.get();
      std::vector<std::string> names;
      mpProp->getPropertyNames(names);
      for (auto& name : names)
        mpSender->declareProperty<std::string>(name,
          std::bind(&SafePropertySender::propertyChanged, pSender, name, std::placeholders::_1),
          std::bind(&SafePropertySender::getProperty, pSender, name));
    }
    return mpSender.get();
  }

  SafePropertyReceiver::GuardLockType SafePropertyReceiver::guard()
  {
    return GuardLockType(mMutex);
  }

  void SafePropertyReceiver::moveGuardToObjectForDeletion(GuardLockType * lock)
  {
    if (lock == nullptr)
      mMutexGuard = GuardLockType(mMutex);
    else mMutexGuard.swap(*lock);
  }

  /** swap the logger instance
  */
  void SafePropertyReceiver::setLogger(PtrToLogger ptr)
  {
    mpLoggerWrapper = ptr;
  }

  void SafePropertyReceiver::setProperty(const std::string& key, const std::string& value)
  {
    mpProp->setProperty(key, value);
  }

  void SafePropertyReceiver::requestProperty(const std::string & key)
  {
    // reads the Property and forward it to the receiver
    std::string value;
    mpProp->getProperty(key, value);
    mpSender->forwardFromReceiver(key, value);
  }

}
}

