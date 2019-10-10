#pragma once
#include "BaseController.h"

#include "MukCommon/MukProblemDefinition.h"
#include "MukCommon/MukVector.h"

namespace gris
{
  namespace muk
  {
    class IStateRegion;
    class VisStateRegion;

    /**
    */
    class ProblemDefinitionController : public BaseController
    {
      public:
        virtual void initialize();
        void setupConnections();

        // adjusting PathCollections and ProblemDefinitions
        void deletePathCollection (const std::string& name);
        void selectPathCollection (const std::string& key);
        
        // adjusting Problem Definition
        void addStartRegion();
        void addWaypoint();
        void addStartRegion(std::unique_ptr<VisStateRegion> pObj);
        void addStartRegion(std::unique_ptr<IStateRegion> pObj);
        void addGoalRegion(std::unique_ptr<VisStateRegion> pObj);
        void addGoalRegion(std::unique_ptr<IStateRegion> pObj);
        void addGoalRegion();
        void deleteRegion();
        void deleteRegion(MukProblemDefinition::EnRegionType type, size_t idx);
        void reverseRegions();

        void initProbDefVisualization();
        void selectVisRegion();
        
        // visualization
        void showProblemDefinitionOnly(const std::string& name);

      private:
        void loadRegion(VisStateRegion& region);

      private:
        const static double MIN_DISTANCE; // minimal distance necessary to be accepted as 'close to picked point'
    };
  }
}
