#pragma once

#include "muk_common_api.h"
#include "MukState.h"

#include "gstd/dynamicProperty.h"

#include <boost/serialization/vector.hpp>

namespace gris
{
  namespace muk
  {
    /** \brief A path with position and direction as a result of path plannign query.

      holds the states of the path as well as several statistics from the planner who created it.
      // code smell: MukPath is a DynamicProperty because of the visualization 
    */
    class MUK_COMMON_API MukPath : public gstd::DynamicProperty
    {
      public:
        MukPath();

      public:
        // old nonsense
        void                          set(MukPath& o);
        void    setRadius(double r)             { mRadius = r; }
        double  getRadius()             const   { return mRadius; }
        void                          setStates(const std::vector<MukState>& states) { mStates = states; }
        const std::vector<MukState>&  getStates() const { return mStates; }
        std::vector<MukState>&        getStates()       { return mStates; }

      public:
        // calculation info
        double    getGoalAngle()            const { return mGoalAngle; }
        double    getGoalDist()             const { return mGoalDist; }
        long long getNumberOfSearchStates() const { return mNumberOfSearchStates; }
        long long getTimeSpend()            const { return mMillisecondsSpend; }
        bool      isApproximated()          const { return mApproximated; }
        void      setGoalAngle(double val)               { mGoalAngle = val; }
        void      setGoalDist(double val)                { mGoalDist = val; }
        void      setNumberOfSearchStates(long long val) { mNumberOfSearchStates = val; }
        void      setTimeSpend(long long val)            { mMillisecondsSpend = val; }
        void      setApproximated(bool b)                { mApproximated = b; }

      public:
        // quickfix for IROS
        void                    setCircularTypes(const std::vector<int>& types)       { mCircularTypes = types; }
        const std::vector<int>& getCircularTypes()                              const { return mCircularTypes; }

      public:
        void swap(MukPath& o);
        
      private:     
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int)
        {
          ar & mApproximated;
          ar & mRadius;
          ar & mGoalDist;
          ar & mGoalAngle;
          ar & mNumberOfSearchStates;
          ar & mMillisecondsSpend;
          ar & mStates;
          ar & mCircularTypes;
        }

      private:
        std::vector<MukState> mStates;
        double mRadius;
        bool   mApproximated;
        double mGoalAngle;
        double mGoalDist;
        long long mNumberOfSearchStates;
        long long mMillisecondsSpend;
        // quickfix for IROS 
        std::vector<int> mCircularTypes; // 0 max circle, 1 min circle
    };

    MUK_COMMON_API void swap(MukPath& l, MukPath& r);
  }
}

namespace std
{
  template <>
  MUK_COMMON_API void swap<gris::muk::MukPath>(gris::muk::MukPath& l, gris::muk::MukPath& r);
}