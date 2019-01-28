#pragma once

namespace gris
{
  namespace muk
  {
    /** \brief relevant values for statistical evaluation
    */
    struct PlannerStatistic
    {
      bool success;
      double goalAngle;
      double goalDist;
      long long millisecondsSpend;
      long long numberOfStatesCreated;
    };
  }
}
