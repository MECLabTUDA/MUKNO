#pragma once

#include <Eigen/Dense>

namespace gris
{
  namespace numeric
  {
    /** \brief Newton-Raphson Method for nonliner systems of equations
      see further: Numerical recipes, p.476
    */
    template<class T, class J>
    class NewtonRaphsonMethod
    {
      public:
       using UsrFun = std::function<void(const T&, T&, J&)>;

      public:
        explicit NewtonRaphsonMethod(UsrFun fun)
          : mFun(fun)
          , mMaxIter(20)
          , mTolX(10e-5)
          , mTolF(10e-5)
        {
        }

        NewtonRaphsonMethod(UsrFun fun, size_t maxIter, double tolx, double tolf)
          : mFun(fun)
          , mMaxIter(maxIter)
          , mTolX(tolx)
          , mTolF(tolf)
        {          
        }

      public:
        static bool solve(T& x, UsrFun fun, size_t ntrial, double tolx, double tolf);
        bool        solve(T& x) { return solve(x, mFun, mMaxIter, mTolX, mTolF); }

        void setMaxIter(size_t maxIter) { mMaxIter = maxIter; }
        void setTolX(double d)          { mTolX = d; }
        void setTolF(double d)          { mTolF = d; }

      private:
        UsrFun mFun;
        double mTolX;
        double mTolF;
        size_t mMaxIter;
    };
  }
}

#include "NewtonRaphsonMethod.hxx"