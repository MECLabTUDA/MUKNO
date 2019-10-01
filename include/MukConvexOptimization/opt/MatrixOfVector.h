#pragma once
#include <Eigen/Core>

#include <functional>
#include <memory>

namespace gris
{
  namespace opt
  {
    /** \brief A function f(x) : R^n -> R^n x R^m, usually the derivative of #VectorOfVector
    */
    class MatrixOfVector
    {
      public:
        typedef std::function<Eigen::MatrixXd(Eigen::VectorXd)> VectorToMatrixFunction;

      public:
        virtual ~MatrixOfVector() {}

        static std::shared_ptr<MatrixOfVector> create(const VectorToMatrixFunction& f);

      public:
        virtual Eigen::MatrixXd operator() (const Eigen::VectorXd& x) const = 0;
        Eigen::MatrixXd         call       (const Eigen::VectorXd& x) const       { return operator()(x); }
    };

    /**
    */
    using MatrixOfVectorPtr = std::shared_ptr<MatrixOfVector>;
  }
}
