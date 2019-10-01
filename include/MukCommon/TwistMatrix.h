#pragma once
#include "LieGroupSE3.h"

namespace gris
{
  namespace se3
  {
    /** \brief A twist in the Lie-Algebra se(3) as homogenous matrix
    */
    class MUK_COMMON_API TwistMatrix
    {
      public:
        TwistMatrix() { mData.setZero(); };
        TwistMatrix(const PoseMatrix& from, const PoseMatrix& to);
        explicit TwistMatrix(const Matrix4d& m) : mData(m) {}
        explicit TwistMatrix(Matrix4d&& m) : mData(m) {}

      public:
        const Matrix4d& data() const  { return mData; }
        Matrix4d& data()              { return mData; }

      public:
        const double& operator()(int i, int j) const { return mData(i,j); }
        double&       operator()(int i, int j)       { return mData(i,j); }

      public:
        TwistVector vee() const;
        PoseMatrix  exp() const;
        PoseMatrix  push(double t) const;

      private:
        Matrix4d mData;
    };
  }
}