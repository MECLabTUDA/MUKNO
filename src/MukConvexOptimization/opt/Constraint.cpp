#include "opt/Constraint.h"

#include <algorithm>
#include <numeric>

namespace gris
{
  namespace opt
  {
    /**
    */
    std::vector<double> Constraint::violations(const std::vector<double>& x) 
    {
      const auto val = value(x);
      std::vector<double> out(val.size());
      if (type() == enEQ)
      {
        for (size_t i(0); i<val.size(); ++i)
          out[i] = fabs(val[i]);
      }
      else
      {
        for (size_t i=0; i < val.size(); ++i) 
          out[i] = std::max(0.0, val[i]);
      }
      return out;
    }

    /**
    */
    double Constraint::violation(const std::vector<double>& x)
    {
      const auto v = violations(x);
      return std::accumulate(v.begin(), v.end(), 0.0);
    }
  }
}