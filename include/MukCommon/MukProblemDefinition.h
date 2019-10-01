#pragma once
#include "Bounds.h"
#include "muk_common.h"
#include "muk_common_api.h"
#include "IStateRegion.h"
#include "StateSamplerData.h"

#include "gstd/DynamicProperty.h"

Forward_Declare_Muk_Boost_Serialization

namespace gris
{
  namespace muk
  {
    /** \brief A representation of a motion planning problem formulation/definition.

      Most importantly, it includes start and goal regions and parameters such as allowed deviation from states, safety distance and the curvature constraint.
    */
    class MUK_COMMON_API MukProblemDefinition : public gstd::DynamicProperty
    {
      public:
        /** \brief types of regions in planning as well as visualization
        */
        enum MUK_COMMON_API EnRegionType
        {
          enNone = -1,
          enStart = 0,
          enWaypoint = 1,
          enGoal = 2
        };

      public:
        MukProblemDefinition();
        MukProblemDefinition(const MukProblemDefinition&) = delete; // no copying when using unique_ptrs
        MukProblemDefinition& operator=(const MukProblemDefinition&) = delete;
        MukProblemDefinition(MukProblemDefinition&&);
        MukProblemDefinition& operator=(MukProblemDefinition&&) = default;
        ~MukProblemDefinition() = default;

      public:
        std::vector<MukState> getStartStates() const;
        std::vector<MukState> getGoalStates()  const;
        std::vector<MukState> getWaypoints()   const;

      public:
        void   setRadius(double val);
        double getRadius()              const  { return mRadius; }
        void   setSafetyDist(double val);
        double getSafetyDist()          const  { return mSafetyDist; }
        void   setKappa(double val);
        double getKappa()               const  { return mKappa; }
        void   setGoalThreshold(double val);
        double getGoalThreshold()       const  { return mGoalThreshold; }
        void   setGoalAngleThreshold(double val);
        double getGoalAngleThreshold()  const  { return mGoalAngleThreshold; }
        void           setBounds(const Bounds& bounds)      { mBounds = bounds; }
        const  Bounds& getBounds()                    const { return mBounds;  }
        void                    setSamplingData(std::unique_ptr<StateSamplerData>&& o) { mpSamplingData = std::move(o); }
        const StateSamplerData* getSamplingData()                                const { return mpSamplingData.get(); };

        void addStartRegion(std::unique_ptr<IStateRegion> mRegion);
        void addGoalRegion (std::unique_ptr<IStateRegion> mRegion);
        IStateRegion& getStartRegion (size_t i) const;
        IStateRegion& getGoalRegion  (size_t i) const;
        void setWaypoint   (const MukState& s, size_t index)  { mWaypoints.at(index) = s; }
        void insertWaypoint(const MukState& s, size_t index);
        void removeWaypoint(size_t index);
        
        void clearStart();
        void clearWaypoints();
        void clearGoal();

        size_t sizeStart()      const { return mStart.size(); }
        size_t sizeGoal()       const { return mGoal.size(); }
        size_t sizeWaypoints()  const { return mWaypoints.size(); }

        void copyParameters(const MukProblemDefinition& other);
        void save(const std::string& filename, unsigned int version) const;
        void load(const std::string& filename, unsigned int version);
        void clone(MukProblemDefinition& target) const;

      private:
        void appendProperties();

      private:
        std::vector<std::unique_ptr<IStateRegion>> mStart;
        std::vector<MukState> mWaypoints;
        std::vector<std::unique_ptr<IStateRegion>> mGoal;

        double               mRadius;
        double               mSafetyDist;
        double               mKappa;
        double               mGoalThreshold;
        double               mGoalAngleThreshold;
        Bounds               mBounds;
        std::unique_ptr<StateSamplerData>   mpSamplingData;
        Use_Muk_Boost_Serialization
    };
  }
}
