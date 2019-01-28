#pragma once

#include "muk_common_api.h"

#include <functional>
#include <string>
#include <ostream>
#include <sstream>

namespace gris
{
  namespace muk
  {
    namespace io
    {
      // LoggerWrapper is not Threadsafe
      class MUK_COMMON_API LoggerWrapper : public std::ostream
      {
      public:
        typedef std::function<void(const std::string&)> CallbackFunction;

      private:
        typedef std::basic_stringbuf<char> BufferType;

      public:
        // build a LoggerWrapper, that passed the logged variables to the callback after destruction
        LoggerWrapper(const CallbackFunction callbackFunction);
        // copy constructor
//        LoggerWrapper(const LoggerWrapper& rhs) : cbFunc(rhs.cbFunc), ostringstream(rhs) {}
        // pass stringstream content to the callback function on destruction
        ~LoggerWrapper();
        // assignment of the content ONLY, the callback-Function is not (and cannot be) changed
        LoggerWrapper& operator=(const LoggerWrapper& rhs);

        virtual LoggerWrapper& flush();
      private:
        const CallbackFunction cbFunc;
        BufferType             mStreamBuffer;
      };
    }
  }
}