#include "opt/ConvexObjective.h"

#include <boost/format.hpp>

#include <iostream>

namespace gris
{
namespace opt
{
  /**
  */
  void ConvexObjective::addAffineExpression(const AffineExpression& affexpr) 
  {
    mQuad += affexpr;
  }

  /**
  */
  void ConvexObjective::addQuadraticExpression(const QuadraticExpression& quadexpr) 
  {
    mQuad += quadexpr;
  }

  /**
  */
  void ConvexObjective::addHinge(const AffineExpression& affexpr, double coeff)
  {
    const auto hinge = mpSolver->addVar("hinge", 0, INFINITY);
    mVariables.push_back(hinge);
    mInequalities.push_back(affexpr);
    mInequalities.back() -= hinge;
    const auto hinge_cost = AffineExpression(hinge) * coeff;
    mQuad += hinge_cost;
  }

  /**
  */
  void ConvexObjective::addAbs(const AffineExpression& affexpr, double coeff)
  {
    auto neg = mpSolver->addVar("neg", 0.0, INFINITY);
    auto pos = mpSolver->addVar("pos", 0.0, INFINITY);
    mVariables.push_back(neg);
    mVariables.push_back(pos);
    AffineExpression neg_plus_pos;
    neg_plus_pos.coeffs() = std::vector<double>(2, coeff);
    neg_plus_pos.vars().push_back(neg);
    neg_plus_pos.vars().push_back(pos);
    mQuad += neg_plus_pos;
    auto affeq = affexpr;
    affeq.vars().push_back(neg);
    affeq.vars().push_back(pos);
    affeq.coeffs().push_back(1);
    affeq.coeffs().push_back(-1);
    mEqualities.push_back(affeq);
  }

  /**
  */
  void ConvexObjective::addHinges(const std::vector<AffineExpression>& ev) 
  {
    for (size_t i=0; i < ev.size(); ++i) 
      addHinge(ev[i],1);
  }

  /**
  */
  void ConvexObjective::addL1Norm(const std::vector<AffineExpression>& ev) 
  {
    for (size_t i=0; i < ev.size(); ++i) 
      addAbs(ev[i],1);
  }

  /**
  */
  void ConvexObjective::addL2Norm(const std::vector<AffineExpression>& ev) 
  {
    for (size_t i=0; i < ev.size(); ++i) 
      mQuad += QuadraticExpression::square(ev[i]);
  }

  /**
  */
  void ConvexObjective::addMax(const std::vector<AffineExpression>& ev) 
  {
    Variable m = mpSolver->addVar("max", -INFINITY, INFINITY);
    for (size_t i=0; i < ev.size(); ++i)
    {
      mInequalities.push_back(ev[i]);
      mInequalities.back() -= m;
    }
  }

  /**
  */
  void ConvexObjective::addConstraintsToModel()
  {
    mConstraints.reserve(mEqualities.size() + mInequalities.size());
    for (const auto& aff : mEqualities)
    {
      const auto* name = "";
      mConstraints.push_back(mpSolver->addEqCnt(aff, name));
    }
    for (const auto& aff : mInequalities)
    {
      const auto* name = "";
      mConstraints.push_back(mpSolver->addIneqCnt(aff, name));
    }
  }

  /**
  */
  void ConvexObjective::removeFromSolver()
  {
    mpSolver->removeCnts(mConstraints);
    mpSolver->removeVars(mVariables);
    mpSolver = nullptr;
  }

  /**
  */
  ConvexObjective::~ConvexObjective()
  {
    if (hasSolver())
      removeFromSolver();
  }

  /**
  */
  double ConvexObjective::value(const std::vector<double>& x) 
  {
    return mQuad.value(x);
  }
}
}