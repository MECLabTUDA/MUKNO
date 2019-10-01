#pragma once
#include "MukCommon/Matrix.h"

namespace gris
{
  namespace opt
  {
    class Variable;
    class AffineExpression;
    class Constraint;
  }

  namespace muk
  {
    using ValueMatrix    = gris::Matrix<double>;
    using VariableMatrix = gris::Matrix<opt::Variable>;
    using AffineExpressionMatrix  = gris::Matrix<opt::AffineExpression>;
    using ConstrMatrix   = gris::Matrix<opt::Constraint>;
  }
}