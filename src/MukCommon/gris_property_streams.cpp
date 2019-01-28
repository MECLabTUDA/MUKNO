#include "private/muk.pch"
#include "gris_property_streams.h"

#include <iterator>

namespace std
{
  template <typename T>
  MUK_COMMON_API ostream& operator<< (ostream& os, const std::vector<T>& v)
  {
    for (size_t i(1); i<v.size(); ++i)
      os << v[i-1] << " ";
    if (!v.empty())
      os << v.back();
    return os;
  }

  template <typename T>
  MUK_COMMON_API istream& operator>> (istream& is, std::vector<T>& v)
  {
    std::string s(std::istreambuf_iterator<char>(is), {});
    std::istringstream iss(s);

    std::copy(std::istream_iterator<int>(iss),
      std::istream_iterator<int>(),
      std::back_inserter(v));

    is.clear(std::ios::goodbit | std::ios::eofbit);  // for boost::lexical_cast
    return is;
  }

  ///**
  //*/
  //ostream& operator<< (ostream& os, const std::vector<int>& v)
  //{
  //  for (size_t i(1); i<v.size(); ++i)
  //    os << v[i-1] << " ";
  //  if (!v.empty())
  //    os << v.back();
  //  return os;
  //}

  ///** 

  //  TODO: learn how these weird istreams work :D
  //*/
  //std::istream& operator>> (std::istream& is, std::vector<int>& v)
  //{
  //  std::string s(std::istreambuf_iterator<char>(is), {});
  //  std::istringstream iss(s);

  //  std::copy(std::istream_iterator<int>(iss),
  //    std::istream_iterator<int>(),
  //    std::back_inserter(v));

  //  is.clear(std::ios::goodbit | std::ios::eofbit);  // for boost::lexical_cast
  //  return is;
  //}
  template MUK_COMMON_API istream& operator>> <int>(istream&, std::vector<int>&);
  template MUK_COMMON_API ostream& operator<< <int>(ostream&, const std::vector<int>&);

  template MUK_COMMON_API istream& operator>> <unsigned int>(istream&, std::vector<unsigned int>&);
  template MUK_COMMON_API ostream& operator<< <unsigned int>(ostream&, const std::vector<unsigned int>&);

  template MUK_COMMON_API istream& operator>> <unsigned short>(istream&, std::vector<unsigned short>&);
  template MUK_COMMON_API ostream& operator<< <unsigned short>(ostream&, const std::vector<unsigned short>&);

  /*template GRIS_GSTD_API Vector<int, double, 3u>;
  template GRIS_GSTD_API Vector<unsigned int, double, 2u>;
  template GRIS_GSTD_API Vector<unsigned int, double, 3u>;*/
}