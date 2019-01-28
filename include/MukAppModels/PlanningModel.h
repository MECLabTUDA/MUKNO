#pragma once
#include "BaseModel.h"

#include "MukCommon/MukScene.h"
#include "MukCommon/StateSamplerData.h"

#include <memory>

namespace gris
{
  namespace muk
  {
    class MukPathGraph;

    /**
    */
    class MUK_APP_API PlanningModel : public BaseModel
    {
      public:
        explicit PlanningModel(std::shared_ptr<MukScene> pScene);
        ~PlanningModel();

      public:
        virtual const char* name() const { return "PlanningModel"; }

      public:
        static const int Invalid_Index = -1;

      public:
        // Path Collection
        void insertPathCollection(const std::string& name);
        void deletePathCollection(const std::string& name);
        void                setActivePathCollection(const std::string& str);
        const std::string&  getActivePathCollection()                         const { return mActivePathCollection; }
        void setActivePathIdx(int idx) { mActivePathIdx = idx; /*emit activePathIndexChanged(); */ }
        int  getActivePathIdx() const { return mActivePathIdx; }
        // MukPaths
        void createPaths(const std::string& pathCollection, size_t numNewPaths);
        void createPaths(const std::string& pathCollection, double timeAvailable);
        void updatePath (const std::string& pathCollection, int pathIdx);
        void loadPath   (const std::string& pathCollection, const std::string& filename);
        void updatePlanner();
        void updatePruner();
        void updateInterpolator(int index, MukPath& path);
        void clearPaths(const std::string& name);
        // Planner etc.
        void setPlanner(const std::string& name);
        void setPruner(const std::string& name);
        void setInterpolator(const std::string&  name);
        // ProblemDefinition
        void setSamplingType(const StateSamplerData& data);
        void updateProblemDefinition(const std::string& name);
        // Search Graph
        std::unique_ptr<MukPathGraph> extractPlanningGraph() const;

      public:
        void setScene(std::shared_ptr<MukScene> pScene)        { mpScene  = pScene; }

      public:
        void configurePlanning(const std::string& name);
        
      private:      
        std::shared_ptr<MukScene>  mpScene;
        std::string mActivePathCollection;
        int         mActivePathIdx;
    };

  }
}
