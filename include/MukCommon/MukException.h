#pragma once

#include "muk_common_api.h"

#include <string>
#include <exception>

namespace gris
{
  namespace muk
  {

    class MUK_COMMON_API MukException : public std::exception
    {
      public:
        enum EnExceptionTypes
        {
          enDefaultError,
          enFileNotFound,
          Size_EnExceptionTypes
        };

      public:
        MukException() {}
        MukException(const char* f, EnExceptionTypes enType = enDefaultError, const char* i = "");
        MukException(const char* f, const char* r = "", const char* i = "")
          : function(f), reason(r), info(i)
        {}
        virtual ~MukException() {}

        virtual char const* what() const;

        const char* getFunction() const { return function.c_str(); }
        const char* getReason()   const { return reason.c_str(); }
        const char* getInfo()     const { return info.c_str(); }

      private:
        mutable std::string mWhat;
        std::string function;
        std::string reason;
        std::string info;
    };

#define MUK_EXCEPTION(what, detail) gris::muk::MukException(__FUNCTION__, what, detail);
#define MUK_EXCEPTION_SIMPLE(what) gris::muk::MukException(__FUNCTION__, what, "");

  }
}
