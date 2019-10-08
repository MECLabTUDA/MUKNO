#include "private/muk.pch"
#include "private/GoalAnalyzer.h"
#include "private/PathEvaluator.h"
#include "SelectionModel.h"

#include "ApplicationModel.h"
#include "PlanningModel.h"
#include "VisualizationModel.h"
#include "WorldVisualizationModel.h"

#include "MukCommon/gris_math.h"
#include "MukCommon/geometry.h"
#include "MukCommon/MukPath.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/PathCollection.h"

#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisPathCollection.h"
#include "MuKVisualization/VisMukPath.h"

class vtkImplicitPlaneWidget2;
class vtkImplicitPlaneRepresentation;

#include <numeric>

namespace
{
  class PaintImageCallback;

  /** \brief selected Indices for drilling
  */
  struct Selection
  {
    std::vector<size_t> selectedIndices;
  };

  using namespace gris::muk;
  /**
  */
  std::vector<size_t> customSort(const std::vector<double>& v, std::function<bool(double, double)> compare);
}

namespace gris
{
  namespace muk
  {
    struct SelectionModel::Impl
    {
      Impl() {}
      ~Impl() {}

      // new data from Moritz' model
      size_t obstacleCount = 12;
      size_t maxComponentCount = 6;
      size_t accessCanalCount = 3;
      const double componentWeightsInitValue  = 0.25;
      const double componentFiltersInitValue  = 100;
      const double obstacleWeightsInitValue   = 0.25;
      const double obstacleFiltersInitValue   = 0.5;
      std::vector<double> componentWeights    = std::vector<double>(maxComponentCount, componentWeightsInitValue);
      std::vector<double> componentFilters    = std::vector<double>(maxComponentCount, componentFiltersInitValue);
      std::vector<double> obstacleWeights     = std::vector<double>(obstacleCount, obstacleWeightsInitValue);
      std::vector<double> obstacleFilters     = std::vector<double>(obstacleCount, obstacleFiltersInitValue);
      // threshold (Values under that th are considered air) 
      const double airThreshold = 500;
      // ratio of pixel under and over the threshold to consider the next state a hole 
      const double holeStartRatio = 0.85;
      // ratio of pixel under and over the threshold to rate the hole as over, thus creating a hysteresis
      const double holeEndRatio = 0.75;
      std::vector<double> holeVariables = { airThreshold, holeStartRatio, holeEndRatio };
      const double        maxDist = std::numeric_limits<double>::infinity();
      MukPath             currentPath;
      bool ctFileLoaded = false;
      bool advancedOptionsRequested = false;
      std::vector<size_t> filteredPaths;
      std::vector<std::string> activeObstacles;

      // old data from Johannes' model
      std::map<std::string, Selection> selections;
      using ActiveCollectionPtr = std::map<std::string, Selection>::const_iterator;
      ActiveCollectionPtr pActive;
      Selection* current = nullptr;
      std::vector<double>  distances;
      std::vector<double>  curvatures;
      std::vector<double>  lengths;
      std::vector<double>  goalAngles;
      std::vector<double>  bonethickness;
      std::vector<double>  airholes;
      std::vector<size_t>  distanceOrder;
      std::vector<size_t>  curvatureOrder;
      std::vector<size_t>  lengthOrder;
      std::vector<size_t>  angleOrder;
      std::vector<size_t>  boneOrder;
      std::vector<size_t>  airholeOrder;
      std::vector<std::vector<double>> minDistToEachObstacle;
      std::vector<std::vector<size_t>> minDistToEachObstacleOrder;
    };

    /**
    */
    SelectionModel::SelectionModel()
      : mp(std::make_unique<Impl>())
    {
    }

    /**
    */
    SelectionModel::~SelectionModel()
    {
    }

    const std::vector<std::vector<double>>& SelectionModel::getMinDistToEachObstacle() const { return mp->minDistToEachObstacle; }
    const std::vector<double>& SelectionModel::getDistances()         const { return mp->distances; }
    const std::vector<double>& SelectionModel::getCurvatures()        const { return mp->curvatures; }
    const std::vector<double>& SelectionModel::getLengths()           const { return mp->lengths; }
    const std::vector<double>& SelectionModel::getGoalAngles()        const { return mp->goalAngles; }
    const std::vector<double>& SelectionModel::getBoneThickness()     const { return mp->bonethickness; }
    const std::vector<double>& SelectionModel::getAirHoles()          const { return mp->airholes; }

    const std::vector<std::vector<size_t>>& SelectionModel::getMinDistToEachObstacleOrder() const { return mp->minDistToEachObstacleOrder; }
    const std::vector<size_t>& SelectionModel::getDistanceOrder()     const { return mp->distanceOrder; }
    const std::vector<size_t>& SelectionModel::getCurvatureOrder()    const { return mp->curvatureOrder; }
    const std::vector<size_t>& SelectionModel::getLengthOrder()       const { return mp->lengthOrder; }
    const std::vector<size_t>& SelectionModel::getAngleOrder()        const { return mp->angleOrder; }
    const std::vector<size_t>& SelectionModel::getBoneOrder()         const { return mp->boneOrder; }
    const std::vector<size_t>& SelectionModel::getAirholesOrder()     const { return mp->airholeOrder; }

    /** \brief Loads internal data of the active path collection. For computation call "compute".
    */
    void SelectionModel::loadPathCollection(const std::string& key)
    {
      auto iter = find_if(mp->selections.begin(), mp->selections.end(), [&] (const auto& sel) { return sel.first == key; });
      if (iter == mp->selections.end())
      {
        mp->current = nullptr;
      }
      else
      {
        mp->current = &iter->second;
        mp->pActive = iter;
      }
    }

    /** returns the Indices of the paths set as the AccessCanals
    */
    std::vector<size_t> SelectionModel::selectedIndices() const
    {
      std::vector<size_t> ret;
      if (mp->current)
        ret = mp->current->selectedIndices;
      return ret;
    }

    /** sets AccessCanal at canalIdx to the path at pathIdx 
    */
    void SelectionModel::setAccessCanal(size_t canalIdx, size_t pathIdx)
    {
      if (!mp->current)
        return;
      // fill with -1 until container is large enough
      auto accessCanals = &mp->current->selectedIndices;
      for (size_t i(accessCanals->size()); i<=canalIdx; ++i)
      {
        accessCanals->push_back(-1);
      }
      bool canBeSet = true;
      for (size_t i(0); i < accessCanals->size(); i++)
      {
        // when you try to set a canal with the same ind it already had it unsets itself instead
        if (pathIdx == accessCanals->at(i) && i == canalIdx)
          canBeSet = false;
        // when a canal should be set to an ind another canal already has it throws an exception
        else if(pathIdx == accessCanals->at(i))
          throw MUK_EXCEPTION_SIMPLE("The path was already set as an AccessCanal elsewhere");
      }
      if (canBeSet)
        accessCanals->at(canalIdx) = pathIdx; //Set the AccessCanal
      else
        accessCanals->at(canalIdx) = -1; //Unset the AccessCanal
    }

    /** determines the (unset) canals that are the most apart at the points where endState is goalThreshold away
    (calculated with the area of an triangle)
    */
    std::vector<size_t> SelectionModel::determineBestCanals(MukState startState)
    {
      auto setCanals = std::vector<bool>(mp->accessCanalCount, false);
      auto accessCanals = mp->current->selectedIndices;
      // checks which canals were already set
      for (size_t i(0); i < accessCanals.size(); i++)
        if (accessCanals[i] != -1)
          setCanals[i] = true;
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      const auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
      auto goalThreshold = mpModels->pAppModel->getScene()->getPathCollection(key).getProblemDefinition()->getGoalThreshold();
      auto pointsOnCircle = std::vector<Vec3d>(mp->filteredPaths.size(), Vec3d(0, 0, 0));
      // gets the state which is goalThreshold away from endState of every path
      for (size_t i(0); i < mp->filteredPaths.size(); i++)
      {
        auto currentPath = pVisColl->getMukPath(mp->filteredPaths[i])->asMukPath().getStates();
        pointsOnCircle[i] = currentPath[determineCutOffState(currentPath, startState, goalThreshold)].coords;
      }
      std::vector<size_t> bestSelection = { 0,0,0};
      auto currentSelection = std::vector<Vec3d>(3, Vec3d(0, 0, 0));
      auto bestArea = 0.0;
      auto currentArea = 0.0;
      // when a Canal was already set the for-loop will run only once with i being the Index of the path set as the Canal
      for (size_t i(setCanals[0] ? accessCanals[0] : 0); i < (setCanals[0] ? accessCanals[0] + 1 : mp->filteredPaths.size()); i++)
      {
        for (size_t j(setCanals[1] ? accessCanals[1] : 0); j < (setCanals[1] ? accessCanals[1] + 1 : mp->filteredPaths.size()); j++)
        {
          // no need to check combination where two indices are the same
          if (j == i)
            continue;
          for (size_t k(setCanals[2] ? accessCanals[2] : 0); k < (setCanals[2] ? accessCanals[2] + 1 : mp->filteredPaths.size()); k++)
          {
            if (k == j || k == i)
              continue;
            //loads i in the first, j in the second and k in the third index of currentSelection
            for (size_t l(0); l < mp->accessCanalCount; l++)
              currentSelection[l] = pointsOnCircle[l == 0 ? mp->filteredPaths[i] : (l == 1 ? j : k)];
            // area of an triangle in 3D: 0.5 * || AB - AC ||
            currentArea = 0.5 * ((currentSelection[1] - currentSelection[0]).cross(currentSelection[2] - currentSelection[0])).norm();
            if (currentArea > bestArea)
            {
              bestArea = currentArea;
              bestSelection = { i,j,k };
            }
          }
        }
      }
      // gives out the Area of the spherical triangle
      auto A = pointsOnCircle[bestSelection[0]];
      auto B = pointsOnCircle[bestSelection[1]];
      auto C = pointsOnCircle[bestSelection[2]];
      auto M = startState.coords;
      auto c = acos((A - M).dot(B - M) / ((A - M).norm() * (B - M).norm())); // the angle <AMB
      auto a = acos((B - M).dot(C - M) / ((B - M).norm() * (C - M).norm())); // the angle <BMC
      auto b = acos((C - M).dot(A - M) / ((C - M).norm() * (A - M).norm())); // the angle <CMA
      auto alpha = acos((cos(a) - cos(b)*cos(c)) / (sin(b)*sin(c))); // the angle <CAB on the sphere
      auto beta = acos((cos(b) - cos(a)*cos(c)) / (sin(a)*sin(c))); // the angle <ABC on the sphere
      auto gamma = acos((cos(c) - cos(a)*cos(b)) / (sin(a)*sin(b))); // the angle <BCA on the sphere 
      auto Area = pow(goalThreshold, 2) * (alpha + beta + gamma - M_Pi); // the Area of the triangle on the sphere
      LOG_LINE << "Area of the spherical triangle: " << Area << " mm² and of the flat triangle: " << bestArea << " mm²";
      // if a canal was set before it wont be set again (as that would unset the canal instead)
      for(size_t i(0); i < mp->accessCanalCount; i++)
        if(!setCanals[i])
          setAccessCanal(i, bestSelection[i]);
      return bestSelection;
    }

    /** determines the state of path at which the endState is cutOffDistance away 
    */
    size_t SelectionModel::determineCutOffState(std::vector<MukState> path, MukState startState, double cutOffDistance)
    {
      double minDist = std::numeric_limits<double>::infinity();
      double currentDist;
      size_t bestIdx;
      for (size_t i(0); i < path.size();i++)
      {
        // Searching for the state that has a distance of cutOffDistance to the startState
        currentDist = abs((startState.coords - path[i].coords).norm() - cutOffDistance);
        if (currentDist < minDist)
        {
          minDist = currentDist;
          bestIdx = i;
        }
      }
      return bestIdx;
    }

    /**
    */
    bool SelectionModel::hasSelection(const std::string& key)
    {
      return std::any_of(mp->selections.begin(), mp->selections.end(), [&] (const auto& sel) { return sel.first == key; });
    }

    /** \brief delete all indices >= maxNumPath
    */
    void SelectionModel::makeSelectionValid(const std::string& key, size_t maxNumPath)
    {
      auto iter = find_if(mp->selections.begin(), mp->selections.end(), [&] (const auto& sel) { return sel.first == key; });
      if (iter == mp->selections.end())
        return;
      auto& v = iter->second.selectedIndices;
      v.erase(std::remove_if(v.begin(), v.end(), [&] (size_t idx) { return idx >= maxNumPath; }), v.end());
    }

    /**
    */
    void SelectionModel::makeSelection(const std::string& key)
    {
      mp->selections.insert(std::make_pair(key, Selection()));
    }

    /**
    */
    void SelectionModel::reset(const std::string& key)
    {
      for (auto& sel : mp->selections)
      {
        sel.second.selectedIndices.clear();
      }
    }

    /** \brief Computes all available properties of existing paths
    */
    void SelectionModel::compute()
    {
      // retrieve necessary data from other models
      const auto& key     = mpModels->pPlanningModel->getActivePathCollection();
      const auto  pScene  = mpModels->pAppModel->getScene();
      const auto& paths   = pScene->getPathCollection(key).getPaths();
      GoalAngleAnalyzer analyzer(*pScene->getPathCollection(key).getProblemDefinition());
      PathEvaluator pEval(mp->componentWeights, mp->obstacleWeights, *pScene->getCollisionDetector(), mp->activeObstacles);

      const auto  N = paths.size();
      if (N==0)
        return;
      auto skip = mpModels->pSelectionModel->selectedIndices();
      // compute parameters
      mp->filteredPaths.clear();
      // fills the filteredPaths with all paths in the beginning as no filters are set yet
      for (size_t i(0); i<N; ++i)
        mp->filteredPaths.push_back(i);
      mp->distances.resize(N);
      mp->curvatures.resize(N);
      mp->goalAngles.resize(N);
      mp->lengths.resize(N);
      // only loads the last two when the ctFile is loaded
      if (mp->ctFileLoaded) 
      {
        mp->bonethickness.resize(N);
        mp->airholes.resize(N);
      }

      // if no visualization is available the VisScene will be empty
      const bool visAvailable = mpModels->pVisModel->getVisScene()->hasPathCollection(key);
      VisPathCollection* pVisColl = visAvailable ? mpModels->pVisModel->getVisScene()->getPathCollection(key).get() : nullptr;
      std::vector<MukPath> interpolatedPaths(N);
      for (int i(0); i < (int)N; ++i)
      {
        if (visAvailable)
          interpolatedPaths[i] = pVisColl->getMukPath(i)->asMukPath();
        else
          mpModels->pPlanningModel->updateInterpolator(i, interpolatedPaths[i]);
      }
#pragma omp parallel for
      for (int i(0); i < (int)N; ++i)
      {
        auto interpolatedPath = interpolatedPaths[i];
        auto dists = computeDistances(*mpModels->pAppModel->getScene()->getCollisionDetector(), interpolatedPath.getStates(), interpolatedPath.getRadius());
        mp->distances[i]  = *std::min_element(dists.begin(), dists.end());
        mp->curvatures[i] = pEval.pathStraightness(interpolatedPath.getStates());
        mp->goalAngles[i] = pEval.goalAngleDifference(paths[i].getStates().back(), pScene->getPathCollection(key).getProblemDefinition()->getGoalStates());
        mp->lengths[i] = computeLength(interpolatedPath.getStates());
        if (mp->ctFileLoaded)
        {
          const auto* pCTImage = mpModels->pWorldVisModel->getCtImage();
          auto currentTextureSpecifics = pEval.textureSpecifics(interpolatedPath.getStates(), pCTImage, interpolatedPath.getRadius(), mp->holeVariables);
          mp->bonethickness[i] = currentTextureSpecifics[1];
          mp->airholes[i] = currentTextureSpecifics[2];
        }
      }
      // only calculates the minDistToEachObstacle when the tab was set as large/window atleast once
      // because it takes alot of time
      if (mp->advancedOptionsRequested)
      {
        if (visAvailable)
        {
          mp->minDistToEachObstacle = pEval.minDistToEachObst(*pVisColl, mp->filteredPaths);
        }
        else
        {
          mp->minDistToEachObstacle = pEval.minDistToEachObst(interpolatedPaths, mp->filteredPaths);
        }
      }
      mp->distanceOrder   = customSort(mp->distances,   std::greater<double>());
      mp->curvatureOrder  = customSort(mp->curvatures,  std::less<double>());
      mp->lengthOrder     = customSort(mp->lengths,     std::less<double>());
      mp->angleOrder      = customSort(mp->goalAngles,  std::less<double>());
      if (mp->ctFileLoaded)
      {
        mp->boneOrder     = customSort(mp->bonethickness, std::less<double>());
        mp->airholeOrder  = customSort(mp->airholes,      std::less<double>());
      }
    }

    /**
    */
    SelectionModel::CurrentBest SelectionModel::getCurrentBest() const
    {
      auto skip = selectedIndices();
      {
        // add the paths that were filtered out to the skip list
        std::vector<size_t> invalidPaths;
        for (size_t i(0); i < mp->distanceOrder.size(); ++i)
        {
          if ((std::find(mp->filteredPaths.begin(), mp->filteredPaths.end(), i) == mp->filteredPaths.end()))
            invalidPaths.push_back(i);
        }
        skip.insert(skip.end(), invalidPaths.begin(), invalidPaths.end());
        // so the upcoming if-clause works
        std::unique(skip.begin(), skip.end());
        // for some reason there might be a -1 in the list
        skip.erase(std::remove_if(skip.begin(), skip.end(), [&] (size_t idx) { return idx == -1; } ), skip.end());
        if (skip.size() == mp->filteredPaths.size())
        {
          throw MUK_EXCEPTION_SIMPLE("no more paths available");
        }
      }
      CurrentBest result;

      auto l_compare = [&] (const size_t idx) { return std::none_of(skip.begin(), skip.end(), [&] (size_t i) {return i==idx;} ); };

      // distances
      auto iter = std::find_if(mp->distanceOrder.begin(), mp->distanceOrder.end(), l_compare);
      result.distance = std::make_pair(*iter, mp->distances[*iter]);
      // curvatures
      iter = std::find_if(mp->curvatureOrder.begin(), mp->curvatureOrder.end(), l_compare);
      result.curvature = std::make_pair(*iter, mp->curvatures[*iter]);
      // angles
      iter = std::find_if(mp->angleOrder.begin(), mp->angleOrder.end(), l_compare);
      result.angle = std::make_pair(*iter, mp->goalAngles[*iter]);
      // lenghts
      iter = std::find_if(mp->lengthOrder.begin(), mp->lengthOrder.end(), l_compare);
      result.length = std::make_pair(*iter, mp->lengths[*iter]);
      if (mp->ctFileLoaded)
      {
        // boneThickness
        iter = std::find_if(mp->boneOrder.begin(), mp->boneOrder.end(), l_compare);
        result.boneThickness = std::make_pair(*iter, mp->bonethickness[*iter]);
        // airholes
        iter = std::find_if(mp->airholeOrder.begin(), mp->airholeOrder.end(), l_compare);
        result.airhole = std::make_pair(*iter, mp->airholes[*iter]);
      }
      return result;
    }

    /**
    */
    SelectionModel::CurrentBest SelectionModel::getIndividual (int idx) const
    {
      CurrentBest result;
      if (idx >= mp->distances.size())
        throw MUK_EXCEPTION("idx exceeds number of available paths", std::to_string(idx).c_str());
      result.distance  = std::make_pair(idx, mp->distances[idx]);
      result.curvature = std::make_pair(idx, mp->curvatures[idx]);
      result.angle     = std::make_pair(idx, mp->goalAngles[idx]);
      result.length    = std::make_pair(idx, mp->lengths[idx]);
      if (mp->ctFileLoaded)
      {
        result.boneThickness = std::make_pair(idx, mp->bonethickness[idx]);
        result.airhole       = std::make_pair(idx, mp->airholes[idx]);
      }
      return result;
    }

    /** returns all paths that fullfil all obstacle and component filters
    */
    std::vector<size_t>& SelectionModel::filterPaths()
    {
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      const auto  pScene = mpModels->pAppModel->getScene();
      const auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
      const auto N = pVisColl->numberOfPaths();
      mp->filteredPaths.clear();
      for (size_t i(0); i < N; ++i)
      {
        bool viableDist = true;
        for (size_t j(0); j < mp->activeObstacles.size(); j++)
        {
          if (roundDouble(mp->minDistToEachObstacle[i][j], 5) < roundDouble(mp->obstacleFilters[j],5))
            viableDist = false;
        }
        if (viableDist                                      &&
          roundDouble(mp->distances[i], 5)  >= roundDouble(mp->componentFilters[0], 5) &&
          roundDouble(mp->curvatures[i], 5) <= roundDouble(mp->componentFilters[1], 5) &&
          roundDouble(mp->goalAngles[i], 5) <= roundDouble(mp->componentFilters[2], 5) &&
          roundDouble(mp->lengths[i], 5)    <= roundDouble(mp->componentFilters[3], 5) &&
          (!mp->ctFileLoaded || // either the ct file is not loaded or the other component filters need to be fulfilled too
            roundDouble(mp->bonethickness[i], 5) <= roundDouble(mp->componentFilters[4], 5) &&
            roundDouble(mp->airholes[i], 5)      <= roundDouble(mp->componentFilters[5], 5)))
          mp->filteredPaths.push_back(i);
      }
      return mp->filteredPaths;
    }



    /** evaluates all filteredPaths according to the component and obstacle weights
    */
    size_t SelectionModel::evaluatePaths()
    {
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      auto*   pScene = mpModels->pAppModel->getScene().get();
      const auto paths = mpModels->pVisModel->getVisScene()->getPathCollection(key);
      double rad = mpModels->pVisModel->getVisScene()->getPathCollection(key)->getMukPath()->asMukPath().getRadius();
      auto numberOfPaths = paths->numberOfPaths();
      if (numberOfPaths == 0)
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
      PathEvaluator pEval(mp->componentWeights, mp->obstacleWeights, *pScene->getCollisionDetector(), mp->activeObstacles);
      // if no weights are applied an exception is thrown
      const auto sumOfWeights = std::accumulate(pEval.getCategoryWeightings().begin(), pEval.getCategoryWeightings().end(), 0.0, [&](const auto& lhs, const auto& rhs) { return lhs + std::abs(rhs); });
      if (sumOfWeights == 0)
      {
        std::string err = "All Component-Weightings are zero, so the path can't be judged";
        std::string info = "Component-Weightings: " + std::to_string(mp->componentWeights[0]) + " " + std::to_string(mp->componentWeights[1]) + " " + std::to_string(mp->componentWeights[2]) + " "
          + std::to_string(mp->componentWeights[3]) + " " + std::to_string(mp->componentWeights[4]) + " " + std::to_string(mp->componentWeights[5]);
        throw MUK_EXCEPTION(err.c_str(), info.c_str());
      }
      std::vector<double> weightedDistances(numberOfPaths);
      // for the distance weight either normal distance or weighted Distance is used depending on if advanced options were requested atleast once
      if (mp->advancedOptionsRequested)
      {
        weightedDistances = pEval.weightedDistances(mp->minDistToEachObstacle, mp->obstacleWeights);
      }
      std::vector<double> weightedDistancesCopy(weightedDistances);
      auto weightedOrder = customSort(weightedDistances, std::greater<double>());
      std::vector<double> distances(numberOfPaths);
      std::vector<double> curvatures(numberOfPaths);
      std::vector<double> goalAngles(numberOfPaths);
      std::vector<double> lengths(numberOfPaths);
      std::vector<double> boneThickness(numberOfPaths);
      std::vector<double> airholes(numberOfPaths);
      auto theBest = getCurrentBest();
      // for every path and every categorie calculates the relation from the pathValue to the best Value (not ideal for airholes because of values of 0)
      for (size_t i(0); i < numberOfPaths; ++i)
      {
        if (mp->advancedOptionsRequested)
        {
          if (i != weightedOrder.front())
            weightedDistances[i] = weightedDistancesCopy[i] / weightedDistancesCopy[weightedOrder.front()] ;
        }
        else
          if (i != theBest.distance.first)
            distances[i] = mp->distances[i] / theBest.distance.second;
        if (i != theBest.curvature.first)
          curvatures[i] = theBest.curvature.second / mp->curvatures[i];
        if (i != theBest.angle.first)
          goalAngles[i] = theBest.angle.second / mp->goalAngles[i];
        if (i != theBest.length.first)
          lengths[i] = theBest.length.second / mp->lengths[i];
        if (mp->ctFileLoaded && i != theBest.boneThickness.first)
          boneThickness[i] = theBest.boneThickness.second / mp->bonethickness[i];
        if (mp->ctFileLoaded && i != theBest.airhole.first)
          airholes[i] = theBest.airhole.second / mp->airholes[i];
      }
      // relation from best to best is 1 ofc
      if (mp->advancedOptionsRequested)
        weightedDistances[weightedOrder.front()] = 1;
      else
        distances[theBest.distance.first] = 1;
      curvatures[theBest.curvature.first] = 1;
      goalAngles[theBest.angle.first] = 1;
      lengths[theBest.length.first] = 1;
      if (mp->ctFileLoaded)
      {
        boneThickness[theBest.boneThickness.first] = 1;
        airholes[theBest.airhole.first] = 1;
      }
      std::vector<double> pathscores(numberOfPaths);
      for (size_t i(0); i < numberOfPaths; ++i)
      {
        std::vector<double> pValues;
        // fills the Values with the ratios if the weights are set
        pValues = { pEval.getCategoryWeightings()[0] > 0 ? (mp->advancedOptionsRequested ? weightedDistances[i] : distances[i]) : 0,
          pEval.getCategoryWeightings()[1] > 0 ? curvatures[i] : 0, 
          pEval.getCategoryWeightings()[2] > 0 ? goalAngles[i] : 0, 
          pEval.getCategoryWeightings()[3] > 0 ? lengths[i]    : 0 };
        if (mp->ctFileLoaded)
        {
          pValues.push_back(pEval.getCategoryWeightings()[4] > 0 ? boneThickness[i] : 0);
          pValues.push_back(pEval.getCategoryWeightings()[5] > 0 ? airholes[i] : 0);
        }
        // save the weighing score of the path
        pathscores[i] = pEval.weigh(pValues, mp->componentWeights);
      }
      // sort the scores from best to worst
      auto pathscoreOrder = customSort(pathscores, std::greater<double>());
      int ret = -1;
      // return the first score from the ordered list that is part of the filteredPaths
      for (size_t i(0); i < pathscoreOrder.size(); ++i)
      {
        for (size_t j(0); j < mp->filteredPaths.size(); ++j)
        {
          if (pathscoreOrder[i] == mp->filteredPaths[j])
          {
            ret = (int)pathscoreOrder[i];
            break;
          }
        }
        if (ret != -1)
          break;
      }
      return ret;
    }

    /**
    */
    void SelectionModel::ctFileLoaded()
    {
      mp->ctFileLoaded = true;
    }

    /**
    */
    void SelectionModel::setComponentWeights(const std::vector<double>& w)
    {
      mp->componentWeights = w;
    }

    std::vector<double>& SelectionModel::getComponentWeights()
    {
      return mp->componentWeights;
    }

    /**
    */
    void SelectionModel::setObstacleWeights(const std::vector<double>& w)
    {
      mp->obstacleWeights = w;
    }

    std::vector<double>& SelectionModel::getObstacleWeights()
    {
      return mp->obstacleWeights;
    }

    void SelectionModel::setComponentFilter(const std::vector<double>& w)
    {
      mp->componentFilters = w;
    }

    std::vector<double>& SelectionModel::getComponentFilter()
    {
      return mp->componentFilters;
    }

    void SelectionModel::setObstacleFilter(const std::vector<double>& w)
    {
      mp->obstacleFilters = w;
    }

    std::vector<double>& SelectionModel::getObstacleFilter()
    {
      return mp->obstacleFilters;
    }

    std::vector<size_t>& SelectionModel::getFilteredPaths()
    {
      return mp->filteredPaths;
    }

    void SelectionModel::setActiveObstacles(const std::vector<std::string>& w)
    {
      mp->activeObstacles = w;
      mp->obstacleCount = w.size();
    }

    void SelectionModel::setAdvancedOptionsRequested(bool requested)
    {
      mp->advancedOptionsRequested = requested;
    }

    bool SelectionModel::getAdvancedOptionsRequested()
    {
      return mp->advancedOptionsRequested;
    }

    /** \brief loads the current chosen path in the cache
    */
    void SelectionModel::loadPath(int idx)
    {
      if (idx < 0)
      {
        mp->currentPath = MukPath();
      }
      else
      {
        const auto& key = mpModels->pPlanningModel->getActivePathCollection();
        // if no visualization is available the VisScene will be empty
        const bool visAvailable = mpModels->pVisModel->getVisScene()->hasPathCollection(key);
        VisPathCollection* pVisColl = visAvailable ? mpModels->pVisModel->getVisScene()->getPathCollection(key).get() : nullptr;
        if (visAvailable)
          mp->currentPath = pVisColl->getMukPath(idx)->asMukPath();
        else
          mpModels->pPlanningModel->updateInterpolator(idx, mp->currentPath);
      }
    }

    /** \brief returns the current path, null if not valid

    used to cache the path of the visualization and get quick access to it
    */
    const MukPath* SelectionModel::getLoadedPath() const
    {
      if (mp->current = nullptr)
      {
        return nullptr;
      }
      return &mp->currentPath;
    }

    /** \utility function to return a double cut of after a given amount of decimals
    */
    double SelectionModel::roundDouble(double value, size_t decimals)
    {
      return (double)((int)(value * pow(10,decimals))) / pow(10, decimals);
    }
  }
}

namespace
{
  /** \brief sorts accourding to?
  */
  std::vector<size_t> customSort(const std::vector<double>& v, std::function<bool(double, double)> compare)
  {
    using Pair = std::pair<size_t, std::vector<double>::const_iterator>;
    std::vector<Pair> order(v.size());
    size_t i(0);
    for (auto it = v.begin(); it != v.end(); ++it, ++i)
      order[i] = make_pair(i, it);

    auto ordering = [&] (const Pair& lhs, const Pair& rhs)
    {
      return compare(*(lhs.second), *(rhs.second));
    };
    std::sort(order.begin(), order.end(), ordering);
    std::vector<size_t> result;
    std::transform(order.begin(), order.end(), back_inserter(result), [&] (const auto& pair) { return pair.first; });
    return result;
  }
}