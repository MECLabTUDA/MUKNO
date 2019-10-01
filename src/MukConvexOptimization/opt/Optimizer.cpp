#include "opt/Optimizer.h"

#include <boost/format.hpp>

#include <sstream>

namespace
{
  std::stringstream toString(const std::vector<double>& v)
  {
    std::stringstream ss;
    for (const auto d : v)
      ss << d << " ";
    return ss;
  }
}

namespace gris
{
namespace opt
{
  /**
  */
  std::ostream& operator<<(std::ostream& o, const OptResults& r) 
  {
    o << "Optimization results:"
      << "\n status:                " << EnOptimizerResultNames[r.status]
      << "\n cost values:           " << toString(r.cost_vals).str()
      << "\n constraint violations: " << toString(r.cnt_viols).str()
      << "\n n func evals:          " << r.n_func_evals
      << "\n n qp solved:           " << r.n_qp_solves;
    return o;
  }

  /**
  */
  void Optimizer::addCallback(const Callback& cb)
  {
    mCallbacks.push_back(cb);
  }

  /**
  */
  void Optimizer::callCallbacks(std::vector<double>& x) 
  {
    for (size_t i=0; i < mCallbacks.size(); ++i)
      mCallbacks[i](mpOptProb.get(), x);
  }

  /**
  */
  void Optimizer::initialize(const std::vector<double>& x)
  {
    if ( ! mpOptProb) 
      throw std::exception("need to set the problem before initializing");
    const auto N1 = mpOptProb->getVars().size();
    const auto N2 = x.size();
    if (N1 != N2) 
      throw std::exception( (boost::format("initialization std::vector has wrong length. expected %i, got %i") % N1 % N2).str().c_str() );
    mResults.clear(); 
    mResults.x = x;
  }

  /**
  */
  void OptResults::clear()
  {
    x.clear();
    status = enBadInitialization;
    improved = false;
    cost_vals.clear();
    cnt_viols.clear();
    n_func_evals = 0;
    n_qp_solves = 0;
  }
}
}