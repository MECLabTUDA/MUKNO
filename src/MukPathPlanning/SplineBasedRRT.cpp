#include "private\muk.pch"
#include "private/SplineBasedRRT.h"

#include "private/IStateValidityChecker.h"
#include "private/MotionValidatorDefault.h"
#include "private/magic_numbers.h"
#include "private/MukGoalStates.h"
#include "private/pathplanning_tools.h"
#include "private/StateSamplerIBounds.h"
#include "private/types.h"

#include "MukCommon/geometry.h"

#pragma warning (push)
#pragma warning( disable: 4267 ) // size_t to unsigned int (x64)
#pragma warning( disable: 4800 ) // forcing value to bool
#pragma warning( disable: 4996 ) // function (localtime) may be unsafe
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/ScopedState.h>
#pragma warning (pop)

#include <Eigen/Dense>

#include "MukCommon/gris_math.h"

#include <limits>
#include <chrono>
#include <functional>

namespace
{
  /** \brief approximation of constant c_4 on page 768, used in eq. (4)
  */
  const double C4 = 1.1228;
}

namespace ompl
{
  namespace geometric
  {
    /**
    */
    SplineBasedRRT::SplineBasedRRT(const base::SpaceInformationPtr &si)
      : base::Planner(si, "SplineBasedRRT")
      , mVerbose(false)
      , mpSampler(nullptr)
      , mpStartGraph(nullptr)
      , mpLastGoalMotion(nullptr)
      , mGoalBias(0.25)
      , mRadiusNearestBall(1.0)      
      , mKappa(0.1)
      , mStepSize(0.0)
    {
      specs_.approximateSolutions = true;
      specs_.directed = false;
    }

    /**
    */
    SplineBasedRRT::~SplineBasedRRT()
    {
      freeMemory();
    }
    
    /** \brief clears memory in the search graph
    */
    void SplineBasedRRT::freeMemory()
    {
      if (mpStartGraph)
      {
        std::vector<Motion*> motions;
        mpStartGraph->list(motions);
        for (auto& pMotion : motions)
        {
          freeMotion(pMotion);
        }
        mpStartGraph->clear();
      }
    }

    /**
    */
    void SplineBasedRRT::freeMotion(Motion* motion) const
    {
      if (motion->pState)
        si_->freeState(motion->pState);
      delete motion;
    }

    /** \brief starts a path computation from scratch

      Checks input, creates additional basic parameters, logs important information.
      Adds initial states, then performs planning.
      Differentiate between approximate and exact solution.
      Creates solution path in the end.
    */
    ompl::base::PlannerStatus SplineBasedRRT::solve(const base::PlannerTerminationCondition &ptc)
    {
      auto timeStart = std::chrono::high_resolution_clock::now();
      if ( ! preProcess())
      {
        return base::PlannerStatus::INVALID_START;
      }
      // prepare helper variables for upcoming loop
      auto* goal       = pdef_->getGoal().get();
      auto* goal_s     = dynamic_cast<base::GoalSampleableRegion*>(goal);
      Motion* pApproxSol = nullptr;
      double  approxDiff = std::numeric_limits<double>::infinity();  // safes distance of state closest to goal
      auto  pSample    = Motion::make_unique(si_);
      auto* rstate     = pSample->pState;
      bool  approximated = false;
      mpFoundSolution  = nullptr;
      
      while (ptc == false && nullptr == mpFoundSolution)
      {       
        // sample random state (with goal biasing)
        auto goalBiased = rng_.uniform01() < mGoalBias;
        if ((goal_s != nullptr) && goalBiased && goal_s->canSample())
          goal_s->sampleGoal(rstate);
        else
          mpSampler->sampleUniform(rstate);

        // find closest state in the tree
        auto nearestBall   = selectNearestNeighbors(pSample.get());
        auto nearestStates = selectStates(nearestBall);
        for (auto* pNearest : nearestStates)
        {
          steer(pNearest, rstate, approxDiff, pApproxSol, ptc);
        }
      }
      // also hold pointer to an approximated solutions
      if (nullptr == mpFoundSolution)
      {
        mpFoundSolution = pApproxSol;
        approximated = true;
      }

      postProcess(true, approximated, approxDiff);

      auto timeEnd = std::chrono::high_resolution_clock::now(); 
      auto diff    = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd-timeStart).count();
      mStats.millisecondsSpend = diff;
      if (mVerbose)
      {
        LOG << "  " << name_ << ": created " << mpStartGraph->size() << " states in " << diff << " milliseconds.\n";
        LOG << "    remaining distance to goal: " << approxDiff << "\n";
        LOG << "    angle to goal:              " << mStats.goalAngle << "\n";
      }
      return base::PlannerStatus(true, approximated);
    }


    /** \brief calculates new states that fulfill the curvature constraint

    calculate the appropriate angle, to move under the curvature constraint from state "from" to state "to"
    due to the curv. constraint a minimum distance has to be calculated, this may be more than the maximum distance (*).

    reference paper: Yang et al., An analytical contiuous-curvature path-smoothing algorithm, IEEE, 2010 

    \param from:  the state, from which the new one is calculated
    \param to:    indicator, that tells the function in which direction the new state should lie, this state therefore will usually not be reached
    \param approxSolution: state, that has been really steered to
    \param approxDiff: remaining distance to goal

    */
    void SplineBasedRRT::steer(Motion* startMotion, const base::State* targetState, double& approxDiff, Motion* approxSolution, const base::PlannerTerminationCondition & ptc)
    {
      using namespace Eigen;
      using namespace gris::muk;

      double newDist = 0;
      double oldDist = std::numeric_limits<double>::infinity();
      Motion* parentMotion = startMotion;
      // iteratively steer until no more progress to 
      while (!ptc && oldDist > newDist && mpFoundSolution == nullptr)
      {
        // location and tangent at state "from"
        const auto* pStart = parentMotion->pState->as<MukStateType>();
        const auto* pGoal  = targetState->as<MukStateType>();
        Vector3d    dirStart = Vector3d(pStart->rotation().x, pStart->rotation().y, pStart->rotation().z);
        Vector3d    dirGoal  = Vector3d(pGoal->getX() - pStart->getX(), pGoal->getY() - pStart->getY(), pGoal->getZ() - pStart->getZ());
        dirStart.normalize();
        const double norm = dirGoal.norm();
        dirGoal /= norm;
        if (dirGoal.hasNaN())
          return;
        // angle between the two tangents
        const double dot = dirStart.dot(dirGoal);
        const double angle = acos(abs(dot));
        const double gamma = dot > 0 ?  angle : gris::M_Pi - angle;
        const bool pointReachable = gamma < mSpiralBuilder.getGamma();
        const double d = mSpiralBuilder.getMinLength();
        // new motion
        auto* nextMotion = Motion::create(si_);
        nextMotion->pParent = parentMotion;
        auto* nextState = nextMotion->pState->as<MukStateType>();
        if (pointReachable) // point reachable
        {
          nextState->setXYZ( pStart->getX() + d*dirGoal.x(), pStart->getY() + d*dirGoal.y(), pStart->getZ() + d*dirGoal.z());
        }
        else
        { 
          // compute most favorable angle, depending of distance to new state
          /*const size_t steps = std::max(1u, static_cast<size_t>( angle / mSpiralBuilder.getGamma() + 0.5 ) );
          const double minApproxLength = steps * mStepSize;*/
          const double minApproxLength = mStepSize;
          //const double newAngle = 0.5 * std::min(1.0, norm / minApproxLength) *  mSpiralBuilder.getGamma();
          const double newAngle = 0.5 * mSpiralBuilder.getGamma();
          // move
          Matrix3d m;
          auto v = dirStart.cross(dirGoal);
          auto binormal = dirStart.cross(v);
          if (dirGoal.dot(binormal) > 0)
            v *= -1;
          v.normalize();
          m = AngleAxisd(newAngle, v);
          dirGoal = m*dirStart;
          dirGoal.normalize();
          if (dirGoal.hasNaN())
          {
            freeMotion(nextMotion);
            return;
          }
          nextState->setXYZ( pStart->getX() + d*dirGoal.x(), pStart->getY() + d*dirGoal.y(), pStart->getZ() + d*dirGoal.z());
        }
        nextState->rotation().x = dirGoal.x();
        nextState->rotation().y = dirGoal.y();
        nextState->rotation().z = dirGoal.z();
        nextState->rotation().w = 0;
        if (motionIsValid(parentMotion->pState, nextMotion->pState))
        {
          mpStartGraph->add(nextMotion);
          parentMotion = nextMotion;
          newDist      = si_->distance(nextMotion->pState, targetState);
          {
            double dist = 0.0;
            bool sat = pdef_->getGoal()->isSatisfied(nextMotion->pState, &dist);
            if (sat)
            {
              approxDiff = dist;
              mpLastGoalMotion = nextMotion;
              mpFoundSolution = nextMotion;
              break;
            }
            if (dist < approxDiff)
            {
              approxDiff = dist;
              approxSolution = nextMotion;
            }
          }
        }
        else
        {
          freeMotion(nextMotion);
          return;
        }
      }
    }
    
    /** \brief performs the necessary build up steps before actual planning algorithm begins
      
      includes e.g. adding the initial states to the search graph
    */
    bool SplineBasedRRT::preProcess()
    {
      checkValidity(); // some internal ompl checks for valid planner state (e.g. assures existence of goal / initial states)

      if ( ! mpSampler)
        mpSampler = si_->allocStateSampler();

      mSpiralBuilder.setKappa(mKappa);
      mSpiralBuilder.setMinLength(0.5*mStepSize);
      const auto gamma = mSpiralBuilder.fromKappaAndMinLength(mKappa, 0.5*mStepSize);
      mSpiralBuilder.setGamma(gamma);

      // add more potential states to the search graph
      while (const base::State *st = pis_.nextStart())
      {
        Motion* motion = Motion::create(si_);
        si_->copyState(motion->pState, st);
        mpStartGraph->add(motion);
      }
      mpFoundSolution = nullptr;
      // throw if there are no initial states
      if (mpStartGraph->size() == 0)
      {
        LOG << "  " << getName().c_str() << ": There are no valid initial states!" << std::endl;
        return false;
      }
      return true;
    }

    /** \brief collects the results from the planning steps

    includes e.g. the building of a solution path, if one was found. Then adds it to the solutions paths in the problem definition.
    */
    void SplineBasedRRT::postProcess(bool solved, bool approximated, double distance)
    {
      mStats.numberOfStatesCreated = mpStartGraph->size();
      mStats.goalDist = distance;
      // at least one valid goal state was found and connected to a initial state
      if (mVerbose)
      {
        LOG << "  " << name_ << ":    remaining distance to goal: " << distance << "\n";
      }
      if ( ! approximated)
      {
        mStats.success = true;
        mStats.goalAngle = dynamic_cast<base::MukGoalStates*>(pdef_->getGoal().get())->angleDistance(mpFoundSolution->pState);
        mpLastGoalMotion = mpFoundSolution;
        if (mVerbose)
        {
          LOG << "    angle to goal:              " << mStats.goalAngle << "\n";
        }
        // recursivley construct the solution path
        std::vector<Motion*> path;
        while (nullptr != mpFoundSolution)
        {
          path.push_back(mpFoundSolution);
          mpFoundSolution = mpFoundSolution->pParent;
        }
        // add the solution path to the problem definition
        auto omplPath = std::make_unique<PathGeometric>(si_);
        for (size_t i = path.size(); i>0; --i)
        {
          omplPath->append(path[i-1]->pState);
        }
        pdef_->addSolutionPath(base::PathPtr(omplPath.release()), approximated, distance, name_);
      }
      else
      {
        mStats.success = false;
      }
    }
    
    /** \brief select the states in the vicinity of a given radius from the state in pMotion

    if empty, returns the single nearest neighbor
    */
    std::vector<Motion*> SplineBasedRRT::selectNearestNeighbors(Motion* pMotion)
    {
      std::vector<Motion*> result;
      mpStartGraph->nearestR(pMotion, mRadiusNearestBall, result);
      if (result.empty())
      {
        result.push_back(mpStartGraph->nearest(pMotion));
      }
      return result;
    }

    /**
    */
    std::vector<Motion*> SplineBasedRRT::selectStates(const std::vector<Motion*>& motions)
    {
      double stepDist = 2.0*mStepSize;
      std::vector<Motion*> result;
      for (auto* pMotion : motions)
      {
        result.push_back(pMotion);
        while (pMotion->pParent)
        {
          pMotion = pMotion->pParent;
          if (distanceFunction(pMotion, result.back()) < stepDist)
          {
            result.push_back(pMotion);
          }
        }
      }
      return result;
    }
    
    /**
    */
    bool SplineBasedRRT::motionIsValid(const base::State* pStart, const base::State* pGoal) const
    {
      using gris::Vec3d;
      using namespace gris::muk;
      auto pWorkMotion = Motion::make_unique(si_);
      ompl::base::State* work = pWorkMotion->pState;      
      si_->copyState(work, pStart);

      using T = MukStateSpace::StateType;

      auto* state = work->as<T>();
      auto* goal  = pGoal->as<T>();
      Vec3d tangent = Vec3d(goal->getX() - state->getX(),
                            goal->getY() - state->getY(),
                            goal->getZ() - state->getZ());
      const size_t N = static_cast<size_t>( ceil(tangent.norm() / gris::planning::Magic_Collision_Query_Resolution) );
      Vec3d step = (1.0 / N) * tangent;
      for (size_t i(0); i<N; ++i)
      {
        if ( ! si_->getStateValidityChecker()->isValid(work))
        {
          return false;
        }        
        state->setX(state->getX()+ step.x());
        state->setY(state->getY()+ step.y());
        state->setZ(state->getZ()+ step.z());
      }
      return true;
    }

    /** \brief
    */
    void SplineBasedRRT::getPlannerData(base::PlannerData &data) const
    {
      Planner::getPlannerData(data);

      std::vector<Motion*> motions;
      if (mpStartGraph)
        mpStartGraph->list(motions);

      if (mpLastGoalMotion)
        data.addGoalVertex(base::PlannerDataVertex(mpLastGoalMotion->pState));

      for (unsigned int i = 0; i < motions.size(); ++i)
      {
        if (nullptr == motions[i]->pParent)
          data.addStartVertex(base::PlannerDataVertex(motions[i]->pState));
        else
          data.addEdge(base::PlannerDataVertex(motions[i]->pParent->pState), base::PlannerDataVertex(motions[i]->pState));
      }
    }

    /** see og::Planner::setup()
    */
    void SplineBasedRRT::setup()
    {
      Planner::setup();
      if ( ! mpSampler)
      {
        mpSampler = si_->allocStateSampler();
      }
      if ( ! mpStartGraph)
      {
        mpStartGraph.reset(new NearestNeighborsSqrtApprox<Motion*>());
      }
      mpStartGraph->setDistanceFunction([&] (const Motion *a, const Motion *b) { return this->distanceFunction(a,b); });
    }

    /** see og::Planner::clear()
    */
    void SplineBasedRRT::clear()
    {
      Planner::clear();
      mpSampler.reset();
      freeMemory();
      mpLastGoalMotion = nullptr;
    }
    
    /**
    */
    void SplineBasedRRT::restartSearch()
    {
      clear();
      pis_.restart(); // planner input states
    }
    
    /** \briefs wraps distance function to distance function of the state space (SE3)
    */
    double SplineBasedRRT::distanceFunction(const Motion *a, const Motion *b) const
    {
      return si_->distance(a->pState, b->pState);
    }

    /** \brief
    */
    void SplineBasedRRT::setStepSize(double val)
    {
      mStepSize = val;
    }
  }
}