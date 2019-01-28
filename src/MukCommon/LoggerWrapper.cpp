#include "private/muk.pch"

#include "LoggerWrapper.h"

#include <iostream>

namespace gris
{
  namespace muk
  {
    namespace io {

      LoggerWrapper::LoggerWrapper(const CallbackFunction callbackFunction)
        : cbFunc(callbackFunction)
        , mStreamBuffer(std::ios_base::out)
        , std::ostream(&mStreamBuffer)
      {}

      LoggerWrapper::~LoggerWrapper()
      {
        flush();
      }

      LoggerWrapper & LoggerWrapper::operator=(const LoggerWrapper & rhs)
      { 
        mStreamBuffer.str(rhs.mStreamBuffer.str()); 
        return *this; 
      }

      LoggerWrapper & LoggerWrapper::flush()
      {
        std::ostream::flush();
        cbFunc(mStreamBuffer.str());
        mStreamBuffer.str("");
        return *this;
      }

    }
  }
}