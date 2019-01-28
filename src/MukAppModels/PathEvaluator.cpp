#include "private/muk.pch"
#include "private/PathEvaluator.h"

#include "MukCommon/geometry.h"
#include "MukCommon/gris_math.h"
#include "MukCommon/ICollisionDetector.h"

#include "MukEvaluation/statistics.h"

#include <vtkImageData.h>
#include <vtkCoordinate.h>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace
{
  using namespace gris;
  using namespace gris::muk;

  Vec3d toWorldCoordinates(const Vec3d& origin, const Vec3d& spacing, const Vec3i& voxel)
  {
    return Vec3d(
        origin.x() + voxel.x()*spacing.x(),
        origin.y() + voxel.y()*spacing.y(),
        origin.z() + voxel.z()*spacing.z()
    );
  }
  Vec3i toVoxelCoordinates(const Vec3d& origin, const Vec3d& spacing, const Vec3d& point)
  {
    return Vec3i(
      (point.x()-origin.x()) / spacing.x(),
      (point.y()-origin.y()) / spacing.y(),
      (point.z()-origin.z()) / spacing.z()
    );
  }
}

namespace gris
{
  namespace muk
  {
    /** \brief

      Assigning the weightings to the categoryWeightings and the obstacleWeightings
    */
    PathEvaluator::PathEvaluator(const std::vector<double>& componentWeights, const std::vector<double>& obstacleWeights, ICollisionDetector& collDet, const std::vector<std::string>& activeObstacles)
    {
      mpCollDet = &collDet;
      // Distance, Straightness, GoalAngle, Texture
      if (componentWeights.empty() || obstacleWeights.empty())
      {
        throw MUK_EXCEPTION_SIMPLE("not enough weights for path evaluation available");
      }
      mpCategoryWeightings.assign(componentWeights.begin(), componentWeights.end());
      mpObstacleWeightings.assign(obstacleWeights.begin(), obstacleWeights.end());
      mpActiveObstacles.assign(activeObstacles.begin(), activeObstacles.end());
    }

    /**
    weighing-function, takes values and weights and combines them to a score
    */
    double PathEvaluator::weigh(const std::vector<double> diffValues, const std::vector<double> diffweightings) const
    {
      double score(0);
      double weightingSumm(0);
      for (size_t i(0); i < diffValues.size(); ++i)
      {
        score += diffValues[i] * diffweightings[i];
        weightingSumm += diffweightings[i];
      }
      return score / weightingSumm;
    }

    /** determines the minimum distance of a path to every (active) obstacle
    */
    std::vector<std::vector<double>> PathEvaluator::minDistToEachObst(VisPathCollection& paths, const std::vector<size_t> filteredPaths)
    {
      // filler for parallel for
      std::vector<double> obstacleFiller(mpActiveObstacles.size(), 999);
      auto minDistToEachObst = std::vector<std::vector<double>>(paths.numberOfPaths(), obstacleFiller);
      std::vector<double> statefiller;
      for (size_t o(0); o < mpActiveObstacles.size(); o++) // for every active Obstacle
      {
        if (mpObstacleWeightings[o] > 0) // that has a weighting greater 0
        {
          for (size_t o2(0); o2 < mpActiveObstacles.size(); o2++) // set all other obstacle inactive
          {
            if (o == o2)
              mpCollDet->setActive(mpActiveObstacles[o2], true);
            else
              mpCollDet->setActive(mpActiveObstacles[o2], false);
          }
          mpCollDet->rebuild(); // rebuild the collision tree
          std::vector<std::vector<double>> pathDistanceToOneObstacle(paths.numberOfPaths(), statefiller);
#pragma omp parallel for
          for (int p(0); p < (int)paths.numberOfPaths(); p++) // determine the distance of that obstacle to every path
          {
            if (std::find(filteredPaths.begin(), filteredPaths.end(), p) != filteredPaths.end()) // of the filtered paths
            {
              pathDistanceToOneObstacle[p] = computeDistances(*mpCollDet, paths.getMukPath(p)->asMukPath().getPath(), paths.getMukPath(p)->asMukPath().getRadius());
              // only save the minimum since thats enough for computation and weighing
              minDistToEachObst[p][o] = *std::min_element(pathDistanceToOneObstacle[p].begin(), pathDistanceToOneObstacle[p].end());
            }
          }
        }
      }
      // rebuild the original tree
      for (auto key : mpActiveObstacles)
      {
        mpCollDet->setActive(key, true);
      }
      mpCollDet->rebuild();
      return minDistToEachObst;
    }

    /** gives every path a weighted obstacle distance score. the more an obstacle is weighed the more the influence it has on the score
    */
    std::vector<double> PathEvaluator::weightedDistances(std::vector<std::vector<double>> minDistToObst, std::vector<double> obstacleWeightings)
    {
      std::vector<double> weightedDistances;
      for (size_t p(0); p < minDistToObst.size(); p++) // for every path
      {
        std::vector<double> obstacleDistances;
        for (size_t o(0); o < minDistToObst[p].size(); o++) // for every obstacle 
        {
          // Influence of distance to an Obstacle is capped at the maximum Safety Distance.
          obstacleDistances.push_back(std::min(maxSafetyDistance, minDistToObst[p][o])); // take the distance of path to obstacle or the max safety distance
        }
        // the distance of a path to each obstacle with the corresponding weights are combined to give a score to the path
        // the path with the best score fits the weightings the most
        weightedDistances.push_back(weigh(obstacleDistances, obstacleWeightings)); // weigh all obstacles in the path with the corresponding obstacle weights
      }
      return weightedDistances;
    }

    /**
    adds all curvatures in a path together in order to compare it to other paths
    */
    double PathEvaluator::pathStraightness(const std::vector<MukState>& path) const
    {
      std::vector<double> Curvatures = computeCurvatures(path);
      double curvaturesCounter(0);
      for (size_t i(0); i < Curvatures.size(); ++i)
        curvaturesCounter += Curvatures[i];
      return curvaturesCounter;
    }

    /**
    calculates the angle between the ideal angle of arrival and the real angle of arrival
    */
    double PathEvaluator::goalAngleDifference(const MukState& pathEndState, const std::vector<MukState>& goalStates) const
    {
      if (goalStates.empty())
        return std::numeric_limits<double>::quiet_NaN();
      double minDist = std::numeric_limits<double>::infinity();
      size_t idx = 0;
      for (size_t i(0); i < goalStates.size(); ++i)
      {
        double d = (goalStates[i].coords - pathEndState.coords).squaredNorm();
        if (d < minDist)
        {
          minDist = d;
          idx = i;
        }
      }
      return wideAngle(goalStates[idx].tangent, pathEndState.tangent) * 360 / M_PI;
    }

    /**
    Analyses the air-to-bone-ratio in the surroundings along the path and sizes airholes along the path.
    Also calculates the bone density along the path but doesn't return it right now. Further testing is needed to see what is important

      \param path
    */
    std::vector<double> PathEvaluator::textureSpecifics(const std::vector<MukState>& path, const vtkImageData* pImage_, const double r, const std::vector<double>& holeVar) const
    {
      vtkImageData* pImage = const_cast<vtkImageData*>(pImage_);
      // determining the transformation of 1 unit in every direction in order to get the stepsize for sweep over the radius of the robot
      //vtkCoordinate p1, p2;
      //pImage->GetScalarComponentAsDouble(1,1,1,0);
      //pImage->GetScalarComponentAsDouble(2,2,2,0);
      const auto origin = Vec3d(pImage->GetOrigin());
      const auto spacing = Vec3d(pImage->GetSpacing());
      const auto minSpacing = *std::max_element(spacing.data(), spacing.data() + 3);
      //const auto dim     = Vec3i (pImage->GetExtent()[1], pImage->GetExtent()[3], pImage->GetExtent()[5]); // dimension

      Vec3i i1, i2;
      auto p1 = toWorldCoordinates(origin, spacing, i1);
      auto p2 = toWorldCoordinates(origin, spacing, i2);
      p2 = p2 - p1;
      double transformationValue = *std::min_element(p2.data(), p2.data() + 3);
      double transformationFactor = 1.0 / minSpacing;
      // AirtoBoneRatio of the path
      double pathAirtoBoneRatio(0);
      // Mean density of the bone along the path
      double pathDensity(0);
      // Biggest airhole along the path
      double biggestAirHole(0);
      // holeStart will be set to the first point thats considered 'air'
      Vec3d holeStart = { 0,0,0 };
      for (size_t i(0); i < path.size(); ++i)
      {
        // counts the pixels around the current state that are under the threshold
        double airCounter(0);
        // mean density of the bone around the current state
        double circleDensity(0);
        // 16 points around the robot are enough because the circle is small
        std::vector<Vec3d> samples(16);
        // saves the already used indices because some will be used more then once
        std::vector<Vec3i> usedIndices;
        // with the calculated transformationFactor it's made sure that no Indices in the radius are missed
        for (size_t j(0); (double)j < (transformationFactor*r) + 1; ++j)
        {
          // circles around the robot with increasing radii acording the stepsize calculated with the transformationFactor
          circleAroundLine(path[i].coords, path[i].tangent, j / transformationFactor, samples);
          for (const auto& sample : samples)
            //for (size_t k(0); k < samples.size(); ++k)
          {
            auto sampleIndex = toVoxelCoordinates(origin, spacing, sample);
            bool used = false;
            // check if index was used before
            for (size_t l(0); l < usedIndices.size(); ++l)
            {
              if (usedIndices[l] == sampleIndex)
              {
                used = true;
                break;
              }
            }
            if (!used)
            {
              // if not used yet, increase the airCounter if the pixelValue is under the defined threshhold 
              // and add the pixelValue to the mean density of the radius around the current state
              auto pixelValue = pImage->GetScalarComponentAsDouble(sampleIndex.x(), sampleIndex.y(), sampleIndex.z(), 0); // component 0 -> 
              if (pixelValue <= holeVar[0])
                ++airCounter;
              circleDensity += pixelValue;
              usedIndices.push_back(sampleIndex);
            }
          }
        }
        // When there has not been a hole and the current ratio of air to bone is over the threshold, mark the current State as the start of an airhole
        if (holeStart.isZero() && airCounter / usedIndices.size() >= holeVar[1])
          holeStart = path[i].coords;
        // When in an hole and the current ratio of air to bone is unter the threshold, mark the airhole as over and set the start back to zero
        if (!holeStart.isZero() && airCounter / usedIndices.size() <= holeVar[2])
          holeStart = { 0,0,0 };
        // Determine the biggest airhole along the path
        double currentHoleLength = (holeStart - path[i].coords).squaredNorm();
        if (!holeStart.isZero() && currentHoleLength > biggestAirHole)
          biggestAirHole = currentHoleLength;
        pathAirtoBoneRatio += airCounter / usedIndices.size();
        circleDensity /= usedIndices.size(); // circle Density can be used for local deviation
        pathDensity += circleDensity;
      }
      pathAirtoBoneRatio /= (double)path.size();
      pathDensity /= (double)path.size();
      
      std::vector<double> result = { pathAirtoBoneRatio, pathDensity, biggestAirHole };
      return result;
    }
  }
}