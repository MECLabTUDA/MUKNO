#pragma once
#include <vector>

namespace gris
{
  /**
  */
  template<class T>
  class Matrix
  {
  public:
    Matrix() 
      : mRows(0)
      , mCols(0)
    {
    }

    Matrix(size_t nRows, size_t nCols) 
      : mRows(nRows)
      , mCols(nCols)
    {
      mData.resize(mRows*mCols, (T)0);
    }

    Matrix(const Matrix& o)
      : mRows(o.mRows)
      , mCols(o.mCols)
      , mData(o.mData)     
    {}

    Matrix& operator=(const Matrix& o) 
    {
      mRows = o.rows(); mCols = o.cols(); mData = o.mData; return *this; 
    }

  public:
    size_t rows() const { return mRows; }
    size_t cols() const { return mCols; }
    size_t size() const { return mData.size(); }

  public:
    const T&  at(size_t row, size_t col)          const   { return mData.at(row * mCols + col); }
    T&        at(size_t row, size_t col)                  { return mData.at(row * mCols + col); }
    const T&  operator()(size_t row, size_t col)  const   { return mData.at(row * mCols + col); }
    T&        operator()(size_t row, size_t col)          { return mData.at(row * mCols + col); }
    T*        data()                                { return mData.data(); }
    T*        data()                        const   { return mData.data(); }
    const std::vector<T>& vectorize()       const   { return mData; }

  public:
    Matrix block(size_t startRow, size_t startCol, size_t nRow, size_t nCol) const 
    {
      Matrix out;
      out.resize(nRow, nCol);
      for (size_t iRow = 0; iRow < nRow; ++iRow)
        for (size_t iCol = 0; iCol < nCol; ++iCol)
          out(iRow, iCol) = at(iRow + startRow, iCol + startCol);
      return out;
    }

    std::vector<T> rblock(size_t startRow, size_t startCol, size_t nCol) const
    {
      std::vector<T> out(nCol);
      for (size_t iCol = 0; iCol < nCol; ++iCol)
        out[iCol] = at(startRow, iCol + startCol);
      return out;
    }

    Matrix middleRows(size_t start, size_t n) const
    {
      Matrix out;
      out.resize(n, mCols);
      for (size_t i = start; i < start + n; ++i)
        for (size_t j = 0; j < mCols; ++j)
          out(i, j) = at(i, j);
      return out;
    }

    Matrix topRows(size_t n)     const {    return middleRows(0, n);  }
    Matrix bottomRows(size_t n)  const {    return middleRows(mRows - n, n);  }

  public:
    std::vector<T> col(size_t col) const
    {
      std::vector<T> out;
      out.reserve(mRows);
      for (size_t row = 0; row < mRows; ++row)
        out.push_back(at(row, col));
      return out;
    }

    std::vector<T> row(size_t row) const
    {
      std::vector<T> out;
      out.reserve(mCols);
      for (size_t col = 0; col < mCols; ++col)
        out.push_back(at(row, col));
      return out;
    }

  public:
    void resize(size_t nRow, size_t nCol) 
    {
      mRows = nRow;
      mCols = nCol;
      mData.resize(mRows * mCols);
    }

  public:
    void setZero()     { for (size_t i(0); i<mData.size(); ++i) mData[i] = (T)0; };
    void setIdentity() { setZero();  for (size_t i(0); i < mRows && i<mCols; ++i) at(i,i) = (T)1; };

  private:
    size_t mRows;
    size_t mCols;
    std::vector<T> mData;
  };
}
