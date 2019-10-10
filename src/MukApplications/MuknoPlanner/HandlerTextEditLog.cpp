#include "private/muk.pch"
#include "private/HandlerTextEditLog.h"

#include "gstd/logger.h"
#include <iostream>

namespace
{
}

namespace gris
{
namespace muk
{
  /**
  */
  HandlerTextEditLog::HandlerTextEditLog(const Builder& builder)
    : mpTextEdit(builder.mpTextEdit)
    , mpWrapper(builder.mpWrapper)
  {
    init();
    setupConnections();
  }

  /**
  */
  HandlerTextEditLog::~HandlerTextEditLog()
  {
  }


  /**
  */
  void HandlerTextEditLog::init()
  {
    mpTextEdit->setReadOnly(true);
  }

  /**
  */
  void HandlerTextEditLog::setupConnections()
  {
    connect(mpWrapper, &QtLogWrapper::incomingMessage, this, &HandlerTextEditLog::incomingMessage);
  }


}
}

