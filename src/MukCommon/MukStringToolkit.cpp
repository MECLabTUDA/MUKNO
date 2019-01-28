#include "private/muk.pch"

#include "MukStringToolkit.h"

#include <boost/tokenizer.hpp>
#include <boost/format.hpp>

namespace gris
{
namespace muk
{

  /** \brief converts string to Vec3d

  input has to be at least 3 floats separated by white spaces

  \param s string like "0.1 0.5 2.5"
  */
  Vec3d stringToVec3d(const std::string& s)
  {
    typedef boost::char_separator<char> Separator;
    typedef boost::tokenizer<Separator> Tokenizer;

    Separator sep(" ");
    Tokenizer tokens(s, sep);
    Vec3d ret;

    auto tok = tokens.begin();
    size_t numTokens = std::distance(tok, tokens.end());
    if (numTokens < 3u)
    {
      throw std::runtime_error( (boost::format("stringtoVec3d: too few floats: %d!") % numTokens).str() );
    }
    ret.x() = ::atof(tok->c_str());
    ++tok;
    ret.y() = ::atof(tok->c_str());
    ++tok;
    ret.z() = ::atof(tok->c_str());
    return ret;
  }

}
}