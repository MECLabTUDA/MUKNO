#pragma once
#include <Eigen/Core>

#include <functional>
#include <memory>

namespace gris
{
  namespace opt
  {
    /** \brief A function f(x) : R^n -> R^n
    */
    class VectorToVectorFunction
    {
      public:
        typedef std::function<Eigen::VectorXd(Eigen::VectorXd)> Function;

      public:
        virtual ~VectorToVectorFunction() {}

        static std::shared_ptr<VectorToVectorFunction> create(const Function&);

      public:
        virtual Eigen::VectorXd operator() (const Eigen::VectorXd& x) const = 0;
        Eigen::VectorXd         call       (const Eigen::VectorXd& x) const      { return operator()(x); }
    };

    /**
    */
    using VectorToVectorFunctionPtr = std::shared_ptr<VectorToVectorFunction>;
  }
}