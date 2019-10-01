#include "private/muk.pch"

#include "opt/GurobiSolver.h"

#include <boost/format.hpp>

#include <assert.h>
#include <cstdio>
#include <functional>
#include <iostream>
#include <map>
#include <sstream>

extern "C" 
{
#include "gurobi_c.h"
}

// for use in private implementation
#define ENSURE_SUCCESS_P(expr) \
do\
{\
    bool error = expr != 0;\
    if (error) \
    {\
      std::string expression = #expr; \
      std::string function   = __FUNCTION__; \
      std::string msg        = GRBgeterrormsg(env()); \
      throw GurobiException(expression, function, msg);\
      /* printf("GRB error: %s while evaluating %s at %s:%i\n", GRBgeterrormsg(env()), #expr, __FILE__,__LINE__);*/ \
    }\
} while(0)

// for use in GurobiSolver
#define ENSURE_SUCCESS(expr) \
do\
{\
    bool error = expr != 0;\
    if (error) \
    {\
      std::string expression = #expr; \
      std::string function   = __FUNCTION__; \
      std::string msg        = GRBgeterrormsg(s_mp->pEnv); \
      throw GurobiException(expression, function, msg);\
    }\
} while(0)

namespace
{
  using namespace gris;
  using namespace gris::opt;

  /**
  */
  void ensureSuccess(int error,GRBenv* pEnv, const char* varname, const char* fnc)
  {
    if (error)
    {
      std::stringstream ss;
      ss << "GRB error: " << GRBgeterrormsg(pEnv) << " while evaluating variable '" << varname << "'in " << fnc << "\n";
      std::cout << ss.str() << std::endl;
      throw std::exception(ss.str().c_str());
    }
  }

  /**
  */
  void resetIndices(std::vector<Variable>& vars)
  {
    for (int i(0); i < static_cast<int>(vars.size()); ++i)
      vars[i].getRep()[i].setIndex(i);
  }

  /**
  */
  void resetIndices(std::vector<Cnt>& cnts)
  {
    for (int i(0); i < static_cast<int>(cnts.size()); ++i)
      cnts[i].getRep()[i].setIndex(i);
  }

  /**
  */
  std::vector<int> vars2inds(const std::vector<Variable>& vars) 
  {
    std::vector<int> inds(vars.size());
    for (size_t i=0; i < inds.size(); ++i)
      inds[i] = vars[i].getRep()->getIndex();
    return inds;
  }

  /**
  */
  std::vector<int> cnts2inds(const std::vector<Cnt>& cnts) 
  {
    std::vector<int> inds(cnts.size());
    for (size_t i=0; i < inds.size(); ++i)
      inds[i] = cnts[i].getRep()->getIndex();
    return inds;  
  }

  /** ?
  */
  void simplify2(std::vector<int>& inds, std::vector<double>& vals)
  {
    using Int2Double = std::map<int, double>;
    Int2Double ind2val;
    for (size_t i=0; i < inds.size(); ++i) 
    {
      ind2val[inds[i]] = 0;
    }
    for (size_t i=0; i < inds.size(); ++i)
    {
      if (vals[i] != 0)
        ind2val[inds[i]] += vals[i];
    }
    inds.resize(ind2val.size());
    vals.resize(ind2val.size());
    int i_new = 0;
    for(const auto& iv : ind2val)
    {
      inds[i_new] = iv.first;
      vals[i_new] = iv.second;
      ++i_new;
    }
  }
}

namespace gris
{
namespace opt
{
  /**
  */
  GurobiException::GurobiException(const std::string& s1, const std::string& s2, const std::string& details)
    : mExpression(s1)
    , mFunction(s2)
    , mDetails(details)
  {
    mWhat = (boost::format("Gurobi-Error occured\n   evaluating %s\n   in function %s\n   detail: %s") % mExpression % mFunction % mDetails).str();
  }

  /**
  */
  std::shared_ptr<GurobiSolver::S_Impl> GurobiSolver::s_mp = nullptr;

  /**
  */
  ConvexOptimizationSolverPtr GurobiSolver::create()
  {
    return std::make_shared<GurobiSolver>();
  }

  /**
  */
  struct GurobiSolver::S_Impl
  {
    S_Impl()
    {
      if (!pEnv)
      {
        GRBloadenv(&pEnv, nullptr);
        ENSURE_SUCCESS(GRBsetintparam(pEnv, "OutputFlag", 0));
      }
    }

    ~S_Impl()
    {
      GRBfreeenv(pEnv);
    }

    GRBenv* pEnv = nullptr;
  };
  
  /**
  */
  struct GurobiSolver::Impl
  {
    Impl(std::shared_ptr<S_Impl>& pObj)
    {
      pEnv = pObj;
      GRBnewmodel(env(), &pModel,"problem",0, nullptr, nullptr, nullptr, nullptr, nullptr);
      /*if (util::GetLogLevel() < util::LevelDebug)
        ENSURE_SUCCESS(GRBsetintparam(gEnv, "OutputFlag", 0));*/
    }

    ~Impl()
    {
      int error = GRBfreemodel(pModel);
      if (error)
      {
        std::cout << "Finalizing Gurobi Library failed" << std::endl;
      }
    }

    GRBenv* env() { return pEnv->pEnv; }

    std::shared_ptr<S_Impl> pEnv;
    GRBmodel* pModel;
  };

  /**
  */
  GurobiSolver::GurobiSolver()
  {
    s_mp = std::make_shared<S_Impl>();
    mp   = std::make_unique<Impl>(s_mp);
  }

  /**
  */
  GurobiSolver::~GurobiSolver()
  {
  }

  /**
  */
  void GurobiSolver::setVerbose(bool b)
  {
    mVerbose = b;
    const char* val = b ? "True" : "False";
    GRBsetparam(mp->env(), "OutputFlag", val);
  }

  /**
  */
  Variable GurobiSolver::addVar(const std::string& name) 
  {
    ENSURE_SUCCESS(GRBaddvar(mp->pModel, 0, nullptr, nullptr, 0.0, -GRB_INFINITY, GRB_INFINITY, GRB_CONTINUOUS, name.c_str()));
    mVariables.push_back(new VariableRepresentation(static_cast<int>(mVariables.size()), name, this));
    return mVariables.back();
  }

  /**
  */
  Variable GurobiSolver::addVar(const std::string& name, double lb, double ub)
  {
    ENSURE_SUCCESS(GRBaddvar(mp->pModel, 0, nullptr, nullptr, 0.0, lb, ub, GRB_CONTINUOUS, const_cast<char*>(name.c_str())));
    mVariables.push_back(new VariableRepresentation(static_cast<int>(mVariables.size()), name, this));
    return mVariables.back();
  }

  /**
  */
  Cnt GurobiSolver::addEqCnt(const AffineExpression& expr, const std::string& name)
  {
    //LOG_TRACE("adding eq constraint: %s = 0", CSTR(expr));
    auto inds = vars2inds(expr.vars());
    auto vals = expr.coeffs();
    simplify2(inds, vals);
    const char* cnt_name = name.empty() ? nullptr : name.c_str();
    ENSURE_SUCCESS(GRBaddconstr(mp->pModel, static_cast<int>(inds.size()), inds.data(), vals.data(), GRB_EQUAL, -expr.constant(), cnt_name));
    /*auto result = GRBaddconstr(mp->pModel, inds.size(), inds.data(), vals.data(), GRB_EQUAL, -expr.constant(), name.c_str());
    ensureSuccess(result, mp->env(), name.c_str(), __FUNCTION__);*/
    mConstraints.push_back(new ConstraintRepresentation(static_cast<int>(mConstraints.size()), this));
    return mConstraints.back();
  }

  /**
  */
  Cnt GurobiSolver::addIneqCnt(const AffineExpression& expr, const std::string& name) 
  {
    auto inds = vars2inds(expr.vars());
    auto vals = expr.coeffs();
    simplify2(inds, vals);
    ENSURE_SUCCESS(GRBaddconstr(mp->pModel, static_cast<int>(inds.size()), inds.data(), vals.data(), GRB_LESS_EQUAL, -expr.constant(), name.c_str()));
    mConstraints.push_back(new ConstraintRepresentation(static_cast<int>(mConstraints.size()), this));
    return mConstraints.back();
  }

  /** \brief why does this return an empty Cnt ?
  */
  Cnt GurobiSolver::addIneqCnt(const QuadraticExpression& qexpr, const std::string& name)
  {
    auto numlnz = static_cast<int>(qexpr.affine().size());
    std::vector<int> linds = vars2inds(qexpr.affine().vars());
    std::vector<double> lvals = qexpr.affine().coeffs();
    std::vector<int> inds1 = vars2inds(qexpr.vars1());
    std::vector<int> inds2 = vars2inds(qexpr.vars2());  
    ENSURE_SUCCESS(GRBaddqconstr(mp->pModel, numlnz, linds.data(), lvals.data(), static_cast<int>(qexpr.size()), 
      inds1.data(), inds2.data(), const_cast<double*>(qexpr.coeffs().data()), 
      GRB_LESS_EQUAL, -qexpr.affine().constant(), name.c_str()));
    return Cnt();
  }

  /**
  */
  void GurobiSolver::removeVars(const std::vector<Variable>& vars)
  {
    auto inds = vars2inds(vars);
    ENSURE_SUCCESS(GRBdelvars(mp->pModel, static_cast<int>(inds.size()), inds.data()));
    for (size_t i=0; i < vars.size(); ++i)
      vars[i].getRep()->setRemoved(true);
  }

  /**
  */
  void GurobiSolver::removeCnts(const std::vector<Cnt>& cnts)
  {
    std::vector<int> inds = cnts2inds(cnts);
    ENSURE_SUCCESS(GRBdelconstrs(mp->pModel, static_cast<int>(inds.size()), inds.data()));
    for (size_t i=0; i < cnts.size(); ++i)
      cnts[i].getRep()->setRemoved(true);
  }

  /**
  */
  void GurobiSolver::setVarBounds(const std::vector<Variable>& vars, const std::vector<double>& lower, const std::vector<double>& upper) 
  {
    assert(vars.size() == lower.size() && vars.size() == upper.size());
    auto inds = vars2inds(vars);
    ENSURE_SUCCESS(GRBsetdblattrlist(mp->pModel, GRB_DBL_ATTR_LB, (int)inds.size(), inds.data(), const_cast<double*>( lower.data()) ));
    ENSURE_SUCCESS(GRBsetdblattrlist(mp->pModel, GRB_DBL_ATTR_UB, (int)inds.size(), inds.data(), const_cast<double*>( upper.data()) ));
  }

  /**
  */
  std::vector<double> GurobiSolver::getVarValues(const std::vector<Variable>& vars) const 
  {
    assert((vars.size() == 0) || (vars[0].getRep()->getIndex()->creator == this));
    auto inds = vars2inds(vars);
    std::vector<double> out(inds.size());
    ENSURE_SUCCESS(GRBgetdblattrlist(mp->pModel, GRB_DBL_ATTR_X, static_cast<int>(inds.size()), inds.data(), out.data()));
    return out;
  }

  /**
  */
  CvxOptStatus GurobiSolver::optimize()
  {
    ENSURE_SUCCESS(GRBoptimize(mp->pModel));
    int status;
    GRBgetintattr(mp->pModel, GRB_INT_ATTR_STATUS, &status);
    if (status == GRB_OPTIMAL)
    {
      double objval; 
      GRBgetdblattr(mp->pModel, GRB_DBL_ATTR_OBJVAL, &objval);
      //LOG_DEBUG("solver objective value: %.3e", objval);
      if (mVerbose)
        LOG_LINE << "solver objective value: "<< objval;
      return enConvexSubProbSolved;
    }
    else if (status == GRB_INFEASIBLE)
    {
      GRBcomputeIIS(mp->pModel);
      return enConvexSubProbInfeasible;
    }
    else
    {
      return enConvexSubProbFailed;
    }
  }

  /**
  */
  //CvxOptStatus GurobiSolver::optimizeFeasRelax()
  //{
  //  double lbpen=GRB_INFINITY, ubpen = GRB_INFINITY,  rhspen=1;
  //  ENSURE_SUCCESS(GRBfeasrelax(mp->pModel, 0/*sum of viol*/, 0/*just minimize cost of viol*/, &lbpen, &ubpen, &rhspen, NULL));
  //  return optimize();
  //}

  /**
  */
  void GurobiSolver::setObjective(const AffineExpression& expr)
  {
    GRBdelq(mp->pModel);

    int nvars;
    GRBgetintattr(mp->pModel, GRB_INT_ATTR_NUMVARS, &nvars);
    assert(nvars == mVariables.size());

    std::vector<double> obj(nvars, 0);
    for (size_t i=0; i < expr.size(); ++i) 
    {
      obj[expr.vars()[i].getRep()->getIndex()] += expr.coeffs()[i];
    }
    ENSURE_SUCCESS(GRBsetdblattrarray(mp->pModel, "Obj", 0, nvars, obj.data()));
    GRBsetdblattr(mp->pModel, "ObjCon", expr.constant());
  }

  /**
  */
  void GurobiSolver::setObjective(const QuadraticExpression& quad_expr) 
  {
    setObjective(quad_expr.affine());
    std::vector<int> inds1 = vars2inds(quad_expr.vars1());
    std::vector<int> inds2 = vars2inds(quad_expr.vars2());
    GRBaddqpterms(mp->pModel, 
      static_cast<int>(quad_expr.coeffs().size()), 
      const_cast<int*>(inds1.data()), 
      const_cast<int*>(inds2.data()), 
      const_cast<double*>(quad_expr.coeffs().data()));
  }

  /**
  */
  void GurobiSolver::update() 
  {
    std::stringstream ss;
    ENSURE_SUCCESS(GRBupdatemodel(mp->pModel));
    {
      int inew = 0;
      for(const auto& var : mVariables) 
      {
        if ( ! var.getRep()->getRemoved()) 
        {
          mVariables[inew] = var;
          const_cast<VariableRepresentation*>(var.getRep())->setIndex(inew);
          ++inew;
        }
        else
        {
          delete var.getRep();
        }
      }
      mVariables.resize(inew);
    }
    {
      int inew = 0;
      for(const auto& cnt : mConstraints)
      {
        if ( ! cnt.getRep()->getRemoved())
        {
          mConstraints[inew] = cnt;
          const_cast<ConstraintRepresentation*>(cnt.getRep())->setIndex(inew);
          ++inew;
        }
        else
        {
          delete cnt.getRep();
        }
      }
      mConstraints.resize(inew);
    }
  }

  /**
  */
  std::vector<Variable> GurobiSolver::getVars() const
  {
    return mVariables;
  }

  /**
  */
  void GurobiSolver::writeToFile(const std::string& fname)
  {
    ENSURE_SUCCESS(GRBwrite(mp->pModel, fname.c_str()));
  }
}
}