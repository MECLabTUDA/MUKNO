#pragma once

#include "MukCommon/MukScene.h"
#include "MukCommon/MukState.h"

#include "MukVisualization/VisPathCollection.h"

class vtkImageData;

#include <array>
#include <vector>

namespace gris
{
  namespace muk
  {
    /** \brief evaluates the cost function for a path
    */
    class PathEvaluator
    {
      public:
        PathEvaluator(const std::vector<double>& componentWeights, const std::vector<double>& obstacleWeights, ICollisionDetector& collDet, const std::vector<std::string>& activeObstacles);

      public:
        // brings together all results of the different categories and weights them against each other
        double weigh(const std::vector<double> diffValues, const std::vector<double> diffweightings) const;
        // calculates the distances of all paths to all different obstacles with differing weightings between those obstacles
        std::vector<double> weightedDistances(std::vector<std::vector<double>> minDistToObst, std::vector<double> obstacleWeightings);

        std::vector<std::vector<double>> minDistToEachObst(VisPathCollection& paths, const std::vector<size_t> filteredPaths);
        std::vector<std::vector<double>> minDistToEachObst(std::vector<MukPath>& paths, const std::vector<size_t> filteredPaths);
        // adds all curvatures of a path 
        double pathStraightness(const std::vector<MukState>& path) const;
        // determines the difference between the ideal angle of arrival and the actual angle of arrival
        double goalAngleDifference(const MukState& pathEndState, const std::vector<MukState>& goalStates) const;
        // analyses the air-to-bone-ratio in the surroundings along the path and recognizes airholes along the path.
        // also calculates the bone density along the path but doesn't return it right now. Further testing is needed to see what is important
        std::vector<double> textureSpecifics(const std::vector<MukState>& path, const vtkImageData* pImage, const double rad, const std::vector<double>& holeVar) const;
        const std::vector<double>& getCategoryWeightings() { return this->mpCategoryWeightings; };
        const std::vector<double>& getObstacleWeightings() { return this->mpObstacleWeightings; };


      private:
        ICollisionDetector* mpCollDet;
        std::vector<double> mpCategoryWeightings; /// saves the weightings for the different categories        
        std::vector<double> mpObstacleWeightings; /// saves the weightings for the different obstacles
        const double maxSafetyDistance = 2.5; // an Obstacle 2,5 mm away is always considered save
        std::vector<std::string> mpActiveObstacles;
    };
  }
}
