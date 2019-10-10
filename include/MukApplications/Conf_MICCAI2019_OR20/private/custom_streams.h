#pragma once

#include <iostream>
#include <vector>

namespace gris
{
  namespace io
  {
    /*template <typename T>
    std::ostream& operator<< (std::ostream& os, const std::vector<T>& v);
    template <typename T>
    std::istream& operator>> (std::istream& is, std::vector<T>& v);*/

    template <typename T>
    std::ostream& operator<< (std::ostream& os, const std::vector<T>& v)
    {
      for (size_t i(1); i<v.size(); ++i)
        os << v[i-1] << " ";
      if (!v.empty())
        os << v.back();
      return os;
    }

    template <typename T>
    std::istream& operator>> (std::istream& is, std::vector<T>& v)
    {
      std::string s(std::istreambuf_iterator<char>(is), {});
      std::istringstream iss(s);

      std::copy(std::istream_iterator<int>(iss),
        std::istream_iterator<int>(),
        std::back_inserter(v));

      is.clear(std::ios::goodbit | std::ios::eofbit);  // for boost::lexical_cast
      return is;
    }
  }
}