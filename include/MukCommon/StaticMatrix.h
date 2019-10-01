#pragma once
#include "gstd/Vector.h"

#include <array>
#include <iomanip>
#include <iostream>
#include <vector>

namespace gris
{
  /** \brief A homogenous transformation matrix

    row-major
  */
  template<class T, size_t ROWS, size_t COLS>
  class StaticMatrix
  {
    public:
      StaticMatrix() { setZero(); }
      StaticMatrix(const StaticMatrix& o)
        : mRows(o.mRows)
        , mCols(o.mCols)
        , mData(o.mData)     
      {}
      StaticMatrix& operator=(const StaticMatrix& o) { mRows = o.rows(); mCols = o.cols(); mData = o.mData; return *this; }

      ~StaticMatrix() {}

    public:
      static StaticMatrix Identity() { StaticMatrix m;  m.setIdentity(); return m; }
      static StaticMatrix Zero()     { StaticMatrix m;  m.setZero();     return m; }

    public:
      size_t rows() const { return mRows; }
      size_t cols() const { return mCols; }
      size_t size() const { return static_cast<int>( mData.size() ); }

    public:
      const T&  at(size_t row, size_t col)          const   { return mData.at(row * mCols + col); }
      T&        at(size_t row, size_t col)                  { return mData.at(row * mCols + col); }
      const T&  operator()(size_t row, size_t col)  const   { return mData.at(row * mCols + col); }
      T&        operator()(size_t row, size_t col)          { return mData.at(row * mCols + col); }
      T*        data()                                      { return mData.data(); }
      T*        data()                              const   { return mData.data(); }

    public:
      void setZero()     { for (size_t i(0); i<mData.size(); ++i) mData[i] = 0; };
      void setIdentity() { setZero();  for (size_t i(0); i < mRows && i<mCols; ++i) at(i,i) = 1; };

    public:
      template<typename SCALAR, typename = std::enable_if_t<std::is_fundamental<SCALAR>::value> >
      StaticMatrix& operator*=(SCALAR d)
      {
        for (size_t i(0); i < mData.size(); ++i)
          mData[i] *= d;
        return *this;
      }

      template<typename SCALAR, typename = std::enable_if_t<std::is_fundamental<SCALAR>::value> >
      friend StaticMatrix operator*(SCALAR d, const StaticMatrix& rhs)
      {
        StaticMatrix ret(rhs);
        ret *= d;
        return ret;
      }

      template<typename SCALAR, typename = std::enable_if_t<std::is_fundamental<SCALAR>::value> >
      friend StaticMatrix operator*(const StaticMatrix& lhs, SCALAR d)
      {
        StaticMatrix ret(lhs);
        ret *= d;
        return ret;
      }

      
      StaticMatrix& operator+=(const StaticMatrix& rhs)
      {
        for (size_t i(0); i < mData.size(); ++i)
        mData[i] += rhs.mData[i];
        return *this;
      }

      friend StaticMatrix operator+(const StaticMatrix& lhs, const StaticMatrix& rhs)
      {
        StaticMatrix ret(lhs);
        ret += rhs;
        return ret;
      }

      friend StaticMatrix operator*(const StaticMatrix& lhs, const StaticMatrix& rhs)
      {
        StaticMatrix ret;
        for (size_t i(0); i<lhs.mRows; ++i)
        {
          for (size_t j(0); j<lhs.mCols; ++j)
          {
            double val(0);
            for (size_t k(0); k < lhs.mCols; ++k)
                val += lhs(i, k) * rhs(k, j);
            ret(i, j) = val;
          }
        }
        return ret;
      }

      template<typename T, typename S>
      friend Vector<T,S,ROWS> operator*(const StaticMatrix& lhs, const Vector<T,S,ROWS> & rhs)
      {
        Vector<T,S,ROWS> ret;
        for (size_t i(0); i<lhs.mRows; ++i)
        {
          double val(0);
          for (size_t j(0); j < lhs.mCols; ++j)
            val += lhs(i, j) * rhs[j];
          ret[i] = val;
        }
        return ret;
      }

    public:
      std::vector<T> col(size_t col) const
      {
        std::vector<T> out;
        out.reserve(mRows);
        for (int row = 0; row < mRows; ++row)
          out.push_back(at(row, col));
        return out;
      }

      std::vector<T> row(size_t row) const
      {
        std::vector<T> out;
        out.reserve(mCols);
        for (int col = 0; col < mCols; ++col)
          out.push_back(at(row, col));
        return out;
      }

    public:
      friend std::ostream& operator<<(std::ostream& os, const StaticMatrix& m)
      {
        for (size_t i(0); i < m.mRows; ++i)
        {
          os << "   "; // prefix
          for (size_t j(0); j < m.mCols; ++j)
            os << " " << std::setprecision(5) << m(i, j);
          if (i+1 < m.mRows)
            os << "\n";
        }
        return os;
      }

    private:
      size_t mRows = ROWS;
      size_t mCols = COLS;
      std::array<T, ROWS*COLS> mData;
  };

  /**
  */
  using Matrix3d   = StaticMatrix<double,3,3>;
  using Matrix4d   = StaticMatrix<double,4,4>;
}