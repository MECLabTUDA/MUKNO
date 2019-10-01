#include "opt/VectorToVectorFunction.h"

namespace gris
{
namespace opt
{
  /**
  */
  std::shared_ptr<VectorToVectorFunction> VectorToVectorFunction::create(const Function& f) 
  {
    struct F : public VectorToVectorFunction 
    {
      F(const Function& _f) : f(_f) {}

      Eigen::VectorXd operator()(const Eigen::VectorXd& x) const 
      {
        return f(x);
      }

      Function f;
    };
    return std::make_shared<F>(f);
  }
}
}