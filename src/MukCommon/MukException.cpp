#include "private/muk.pch"
#include "MukException.h"

#include <boost/format.hpp>

namespace
{

  const char* String_EnExceptionTypes[gris::muk::MukException::Size_EnExceptionTypes]
  {
    "File not found",
  };
}

namespace gris
{
namespace muk
{

  MukException::MukException(const char* f, EnExceptionTypes enType, const char* i)
    : function(f)    
    , info(i)
  {
    if (enType >= Size_EnExceptionTypes)     
    {
      reason = "Internal Buffer Overflow inside Exception";
    }
    else
    {
      reason = String_EnExceptionTypes[enType];
    }
  }

  char const*  MukException::what() const 
  {
    mWhat = (boost::format("%s: %s (%s)") % function % reason % info).str();
    return mWhat.c_str();
  }

}
}