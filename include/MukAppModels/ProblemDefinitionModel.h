#pragma once
#include "BaseModel.h"

#include "MukCommon/Waypoints.h"
#include "MukCommon/MukProblemDefinition.h"
#include "MukCommon/MukScene.h"

namespace gris
{
  namespace muk
  {
    class VisWaypoints;
    class VisStateRegion;
    class TabPlanning;
    class MukProblemDefinition;
    class VisPathCollection;

    /** \brief Handles the insertion of start and goal regions
    */
    class MUK_APP_API ProblemDefinitionModel : public BaseModel
    {
      public:
        ProblemDefinitionModel();

      public:
        virtual const char* name() const { return "ProblemDefinitionModel"; }

      public:
        void setScene(std::shared_ptr<MukScene> pScene) { mpScene = pScene; }
        
        void unloadAll();
        
      public:
        const std::string&  getKey()        const { return mKey;   }
        int                 getIndex()      const { return mIndex; }

        void loadPathCollection(const std::string& key);

        bool addStartRegion();
        bool addGoalRegion();
        bool addStartRegion(std::unique_ptr<VisStateRegion> pObj);
        bool addGoalRegion(std::unique_ptr<VisStateRegion> pObj);
        bool deleteRegion(MukProblemDefinition::EnRegionType en, int index);
        void reverseRegions();

      private:
        std::shared_ptr<MukScene> mpScene;
        TabPlanning*      mpTabPlanning;
        
        std::string mKey;
        int         mIndex;
        
        VisPathCollection* mpVisColl;
        //VisStateRegion* mpVisRegion;
    };
  }
}
