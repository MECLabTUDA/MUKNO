#pragma once

#include "muk_common_api.h"
#include "MukState.h"

#include "gstd/dynamicProperty.h"

namespace gris
{
  namespace muk
  {
    /** \brief A path in SE3 as the result of a path planning query.

      holds the states of the path as well as several statistics from the planner who created it.
    */
    class MUK_COMMON_API MukPath : public gstd::DynamicProperty
    {
      public:
        MukPath();

      public:
        void                          set(MukPath& o);

        void    setRadius(double r)             { mRadius = r; }
        double  getRadius()             const   { return mRadius; }
                
        void                          setPath(const std::vector<MukState>& path);
        const std::vector<MukState>&  getPath() const   { return mStates; }
        std::vector<MukState>&        getPath()         { return mStates; }

        void                          setStates(const std::vector<MukState>& states) { mStates = states; }
        const std::vector<MukState>&  getStates() const { return mStates; }
        std::vector<MukState>&        getStates()       { return mStates; }

        // calculation info
        double    getGoalAngle()            const { return mGoalAngle; }
        double    getGoalDist()             const { return mGoalDist; }
        long long getNumberOfSearchStates() const { return mNumberOfSearchStates; }
        long long getTimeSpend()            const { return mMillisecondsSpend; }
        bool      isApproximated()          const { return mApproximated; }
        
        void setGoalAngle(double val)               { mGoalAngle = val; }
        void setGoalDist(double val)                { mGoalDist = val; }
        void setNumberOfSearchStates(long long val) { mNumberOfSearchStates = val; }
        void setTimeSpend(long long val)            { mMillisecondsSpend = val; }
        void setApproximated(bool b)                { mApproximated = b; }

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
        }

      private:
        std::vector<MukState> mStates;
        double mRadius;
        bool   mApproximated;
        double mGoalAngle;
        double mGoalDist;
        long long mNumberOfSearchStates;
        long long mMillisecondsSpend;
    };

    MUK_COMMON_API void swap(MukPath& l, MukPath& r);

  }
}

namespace std
{
  template <>
  MUK_COMMON_API void swap<gris::muk::MukPath>(gris::muk::MukPath& l, gris::muk::MukPath& r);
}