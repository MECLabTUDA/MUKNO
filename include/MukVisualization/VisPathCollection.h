#pragma once
#include "VisMukPath.h"
#include "VisStateRegion.h"
#include "VisWaypoints.h"

namespace gris
{
  namespace muk
  {
    class PathCollection;

    /**
    */
    class MUK_VIS_API VisPathCollection
    {
      public:
        explicit VisPathCollection(const std::string& name);
        ~VisPathCollection();

      public:
        const std::string& getName() const { return mName; }

        void addStartRegion(std::shared_ptr<VisStateRegion> pVis);
        void addGoalRegion (std::shared_ptr<VisStateRegion> pVis);
        //void addWaypoint   (std::shared_ptr<VisStateRegion> pVis);

        size_t sizeStart() const { return mStartRegions.size(); };
        size_t sizeGoal()  const { return mGoalRegions.size(); };
        //size_t sizeWaypoints() const { return mpWaypoints.size(); };

        VisStateRegion& getStartRegion(size_t i) const;
        VisStateRegion& getGoalRegion (size_t i) const;
        //VisStateRegion& getWaypoint   (size_t i) const;

        std::vector<std::shared_ptr<VisStateRegion>> getStartRegions() const { return mStartRegions; }
        std::vector<std::shared_ptr<VisStateRegion>> getGoalRegions()  const { return mGoalRegions; }
        //std::vector<std::shared_ptr<VisStateRegion>> getWaypoints   () const;

        void deleteStartRegion(size_t i);
        void deleteGoalRegion (size_t i);
        //void deleteWaypoint   (size_t i);
        
        void addMukPath(std::shared_ptr<VisMukPath> pVis);
        std::shared_ptr<VisMukPath>     getMukPath(size_t idx=0);
        void deleteMukPath(size_t idx);
        
      public:
        size_t numberOfPaths() const           { return mPaths.size(); }
        
      private:
        std::string mName;
        std::vector<std::shared_ptr<VisStateRegion>> mWaypoints;
        std::vector<std::shared_ptr<VisStateRegion>> mStartRegions;
        std::vector<std::shared_ptr<VisStateRegion>> mGoalRegions;
        std::vector<std::shared_ptr<VisMukPath>> mPaths;
    };

  }
}
