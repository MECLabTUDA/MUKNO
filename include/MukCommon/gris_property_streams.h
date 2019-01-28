#pragma once
#include "muk_common_api.h"

#include <iostream>
#include <vector>

namespace std
{
  template <typename T>
  MUK_COMMON_API ostream& operator<< (ostream& os, const std::vector<T>& v);
  template <typename T>
  MUK_COMMON_API istream& operator>> (istream& is, std::vector<T>& v);


  /*MUK_COMMON_API ostream& operator<< (ostream& os, const std::vector<int>& v);
  MUK_COMMON_API istream& operator>> (istream& is, std::vector<int>& v);

  MUK_COMMON_API ostream& operator<< (ostream& os, const std::vector<unsigned int>& v);
  MUK_COMMON_API istream& operator>> (istream& is, std::vector<unsigned int>& v);*/
}

namespace gris
{
  namespace muk
  {
    //using std::istream& std::operator>> (std::istream& is, std::vector<int>& v);
  }
}