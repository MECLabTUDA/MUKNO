#pragma once
#include "muk_common_api.h"

#include <boost/lexical_cast.hpp>

#include <iostream>

namespace gris
{
  namespace muk
  {
    /** \brief A struct thats allows converting a boolean string (true, false) into a boolena variable via boost::lexical_cast

      sample code:
        #include <boost/lexical_cast.hpp>

        const auto str = std::string("test");
        const bool var = boost::lexical_cast<LocaleBool>(str);
    */
    struct MUK_COMMON_API LocaleBool 
    {
      public:
        LocaleBool() {}
        LocaleBool( bool b) : mData(b) {}

      public:
        operator bool() const { return mData; }
        friend std::ostream & operator << ( std::ostream& out, LocaleBool b )
        {
          out << std::boolalpha << b.mData;
          return out;
        }
        friend std::istream & operator >> ( std::istream& in, LocaleBool& b )
        {
          in >> std::boolalpha >> b.mData;
          return in;
        }

      public:
        bool mData;
    };
  }
}