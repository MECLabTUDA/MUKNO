#pragma once
#include "BaseModel.h"
#include <PlanningModel.h>

#include <MukCommon/MukScene.h>

#include <MukEvaluation/statistics.h>

#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisMukPath.h"

#include "gstd/DynamicProperty.h"

#include <memory>
#include <map>
#include <string>

namespace gris
{
	namespace muk
	{
		class MukQMenuBar;
		class MukQToolBar;
		class MuknoPlannerMainWindow;
    class VisScene;
    class TabSelection;

    /** \brief performs the selection process. Provides functions for visualization of differences between paths and highlighting of specific paths.
    */
		class MUK_APP_API SelectionModel : public BaseModel
		{
      public:
        struct CurrentBest
        {
          using Pair = std::pair<size_t, double>;
          Pair distance;
          Pair curvature;
          Pair angle;
          Pair length;
          Pair boneThickness;
          Pair airhole;
        };

			public:
				SelectionModel();
        virtual ~SelectionModel();

      public:
        virtual const char* name() const { return "SelectionModel"; }
				
      public:
        void loadPathCollection(const std::string&);
        void setAccessCanal(size_t canalIdx, size_t pathIdx);
        std::vector<size_t> determineBestCanals(MukState startState);
        size_t determineCutOffState(std::vector<MukState> path, MukState startState, double cutOffDistance);
        
        bool hasSelection(const std::string& key);
        void makeSelectionValid(const std::string& key, size_t numMaxPaths);
        void makeSelection(const std::string& key);
        void reset(const std::string& key);
        std::vector<size_t> selectedIndices() const;

        const std::vector<double>& getDistances() const;
        const std::vector<double>& getCurvatures() const;
        const std::vector<double>& getLengths() const;
        const std::vector<double>& getGoalAngles() const;
        const std::vector<double>& getBoneThickness() const;
        const std::vector<double>& getAirHoles() const;

        const std::vector<size_t>& getDistanceOrder() const;
        const std::vector<size_t>& getCurvatureOrder() const;
        const std::vector<size_t>& getLengthOrder() const;
        const std::vector<size_t>& getAngleOrder() const;
        const std::vector<size_t>& getBoneOrder() const;
        const std::vector<size_t>& getAirholesOrder() const;

        void compute();
        CurrentBest getCurrentBest() const;
        CurrentBest getIndividual (int idx) const;

        // Moritz's new stuff
        void ctFileLoaded();
        void setComponentWeights(const std::vector<double>& w);
        std::vector<double>& getComponentWeights();
        void setObstacleWeights (const std::vector<double>& w);
        std::vector<double>& getObstacleWeights();
        void setComponentFilter(const std::vector<double>& w);
        std::vector<double>& getComponentFilter();
        void setObstacleFilter(const std::vector<double>& w);
        std::vector<double>& getObstacleFilter();
        std::vector<size_t>& getFilteredPaths();
        void setActiveObstacles(const std::vector<std::string>& w);
        void setAdvancedOptionsRequested(bool requested);
        bool getAdvancedOptionsRequested();
        std::vector<std::vector<double>>& getMinDistToEachObst();
        std::vector<size_t>& filterPaths();
        size_t  evaluatePaths();
        void           loadPath(int idx);
        const MukPath* getLoadedPath() const;

        double roundDouble(double value, size_t decimals);
        
        
			private:
        struct Impl;
        std::unique_ptr<Impl> mp;
		};
	}
}