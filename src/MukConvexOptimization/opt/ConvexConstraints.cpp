#include "opt/ConvexConstraints.h"
#include "opt/ConvexOptimizationSolver.h"

#include <algorithm>
#include <numeric>

namespace gris
{
namespace opt
{
  /**
  */
  void ConvexConstraints::addEqCnt(const AffineExpression& aff) 
  {
    mEqualities.push_back(aff);
  }

  /**
  */
  void ConvexConstraints::addIneqCnt(const AffineExpression& aff) 
  {
    mInequalities.push_back(aff);
  }

  /**
  */
  void ConvexConstraints::addConstraintsToModel() 
  {
    mConstraints.reserve(mEqualities.size() + mInequalities.size());
    for (const auto& aff : mEqualities)
    {
      //const auto* name = (boost::format(__eq_%02i) % counter++).str().c_str();
      const auto* name = "";
      mConstraints.push_back(mpModel->addEqCnt(aff, name));
    }
    for (const auto& aff : mInequalities)
    {
      //const auto* name = (boost::format(__ineq_%02i) % counter++).str().c_str();
      const auto* name = "";
      mConstraints.push_back(mpModel->addIneqCnt(aff, name));
    }
  }

  /**
  */
  void ConvexConstraints::removeFromModel()
  {
    mpModel->removeCnts(mConstraints);
    mpModel = nullptr;
  }

  /**
  */
  std::vector<double> ConvexConstraints::violations(const std::vector<double>& x) 
  {
    std::vector<double> out;
    out.reserve(mEqualities.size() + mInequalities.size());
    for(const auto& aff : mEqualities)
      out.push_back(fabs(aff.value(x.data())));
    for(const auto& aff : mInequalities) 
      out.push_back( std::max(0.0, aff.value(x.data())) );
    return out;
  }

  /**
  */
  double ConvexConstraints::violation(const std::vector<double>& x)
  {
    const auto v = violations(x);
    return std::accumulate(v.begin(), v.end(), 0.0);
  }

  /**
  */
  ConvexConstraints::~ConvexConstraints()
  {
    if (inModel()) 
      removeFromModel();
  }
}
}