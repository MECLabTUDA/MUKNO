#include "private\muk.pch"

#include "private/SplineBasedRRTconnect.h"
#include "private/StateSamplerIBounds.h"
#include "private/IStateValidityChecker.h"
#include "private/MotionValidatorDefault.h"
#include "private/types.h"
#include "private/pathplanning_tools.h"
#include "private/MukGoalStates.h"

#include "MukCommon/geometry.h"
#include "MukCommon/gris_math.h"

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

  void approximateStepSize(double eps, double minGamma, double maxGamma, double stepSize, gris::muk::BezierSpiralInterpolator& spiralBuilder);

  gris::Vec3d positionAsVec3d(const ompl::geometric::Motion* motion)
  {
    auto* state = motion->pState->as<ompl::geometric::MukStateType>();
    return gris::Vec3d(state->getX(), state->getY(), state->getZ());
  }
  Eigen::Vector3d positionAsEigen(const ompl::base::State* input)
  {
    auto* state = input->as<ompl::geometric::MukStateType>();
    return Eigen::Vector3d(state->getX(), state->getY(), state->getZ());
  }
  Eigen::Vector3d directionAsEigen(const ompl::base::State* input)
  {
    auto* state = input->as<ompl::geometric::MukStateType>();
    return Eigen::Vector3d(state->rotation().x, state->rotation().y, state->rotation().z).normalized();
  }
}

namespace ompl
{
  namespace geometric
  {
    /**
    */
    SplineBasedRRTconnect::SplineBasedRRTconnect(const base::SpaceInformationPtr &si)
      : base::Planner(si, "SplineBasedRRTconnect")
      , mpSampler(nullptr)
      , mpStartTree(nullptr)
      , mpGoalTree(nullptr)
      , mpLastGoalMotion(nullptr)
      , mpLastStartMotion(nullptr)
      , mGoalBias(0.25)
      , mRadiusNearestBall(1.0)
      , mKappa(0.1)
      , mStepSize(0.0)
    {
      specs_.approximateSolutions = true;
      specs_.directed = false;
      mSpiralBuilder.setKappa(mKappa);
    }

    /**
    */
    SplineBasedRRTconnect::~SplineBasedRRTconnect()
    {
      freeMemory();
    }

    /**
    */
    void SplineBasedRRTconnect::freeMotion(Motion* motion) const
    {
      if (motion->pState)
        si_->freeState(motion->pState);
      delete motion;
    }

    /** \brief clears memory in the search graph
    */
    void SplineBasedRRTconnect::freeMemory()
    {
      std::vector<Motion*> motions;
      if (mpStartTree)
      {
        mpStartTree->list(motions);
        for (auto& pMotion : motions)
        {
          freeMotion(pMotion);
        }
        mpStartTree->clear();
      }
      if (mpGoalTree)
      {
        mpGoalTree->list(motions);
        for (auto& pMotion : motions)
        {
          freeMotion(pMotion);
        }
        mpGoalTree->clear();
      }
      if (mpConnectionAttempts)
      {
        mpConnectionAttempts->list(motions);
        for (auto& pMotion : motions)
        {
          freeMotion(pMotion);
        }
        mpConnectionAttempts->clear();
      }
    }

    /** \brief starts a path computation from scratch

      Checks input, creates additional basic parameters, logs important information.
      Adds initial/goal states, then performs planning.
      Differentiate between approximate and exact solution.
      Creates solution path in the end.
    */
    ompl::base::PlannerStatus SplineBasedRRTconnect::solve(const base::PlannerTerminationCondition &ptc)
    {
      // measure whole process
      auto timeStart = std::chrono::high_resolution_clock::now();
      // prepare search graphs and other private members
      if ( ! preprocess())
      {
        return base::PlannerStatus::INVALID_START;
      }

      //// initialize helper variables for upcoming loop
      auto* goal       = pdef_->getGoal().get();
      auto* goal_s     = dynamic_cast<base::GoalSampleableRegion*>(goal);
      bool  disconnected  = true;
      auto  pSample       = Motion::make_unique(si_);
      auto* rstate     = pSample->pState;
      auto  mUseStartTree = false;
      mConnectionPoint = std::make_pair(nullptr, nullptr);
      mpLastGoalMotion = nullptr;
      mpLastStartMotion = nullptr;

      // go for it
      while (ptc == false && disconnected)
      {
        mUseStartTree = !mUseStartTree;
        TreeType& treeFrom = mUseStartTree ? *mpStartTree : *mpGoalTree;
        TreeType& treeTo   = mUseStartTree ? *mpGoalTree  : *mpStartTree;
        
        // sample random state (with goal biasing)
        /*auto goalBiased = rng_.uniform01() < mGoalBias;
        if ((goal_s != nullptr) && goalBiased && goal_s->canSample())
          goal_s->sampleGoal(rstate);
        else*/
          mpSampler->sampleUniform(rstate);

        auto nearestBall   = selectNearestNeighbors(treeFrom, pSample.get());
        auto nearestStates = selectStates(nearestBall);

        std::vector<Motion*> newMotions;
        for (auto* pNearest : nearestStates)
        {
          // create new states in treeFrom
          auto motion = steer(treeFrom, pNearest, rstate, ptc);
          if(nullptr != motion)
            newMotions.push_back(motion);
        }

        for (size_t i(0); i<newMotions.size(); ++i)
        {
          if (ptc == true || ! disconnected)
            break;
          std::vector<Motion*> otherMotions = selectStatesFromOtherTree(treeTo, newMotions[i]);
          for (auto* other : otherMotions)
          {
            if (connectTree(treeFrom, newMotions[i], treeTo, other, ptc))
            {
              disconnected = false;
              if ( ! mUseStartTree)
                std::swap(mConnectionPoint.first, mConnectionPoint.second);
              break;
            }
          }
        }
      }
      // check success and prepare solution
      postprocess(true, disconnected);
      auto timeEnd = std::chrono::high_resolution_clock::now();
      auto diff    = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd-timeStart).count();
      mStats.millisecondsSpend = diff;
      LOG << "  " << name_ << ": created " << mpStartTree->size() + mpGoalTree->size() << " states in " << diff << " milliseconds.\n"
        << "    Created " << mpStartTree->size() << " states from initial states.\n"
        << "    Created " << mpGoalTree->size()  << " states from goal states.\n" ;
      LOG_LINE;

      return base::PlannerStatus(true, disconnected);
    }

    /** \brief performs the necessary build up steps before actual planning algorithm begins

    includes e.g. adding the initial states to the search graph #mpStartTree
    adding the goal states to the search graph #mpGoalTree
    */
    bool SplineBasedRRTconnect::preprocess()
    {
      // some internal ompl checks for valid planner state (e.g. assures existence of goal / initial states)
      checkValidity();
      if (!mpSampler)
        mpSampler = si_->allocStateSampler();
      // add more potential states to the search graph #mpStartTree
      while (const base::State *st = pis_.nextStart())
      {
        Motion* motion = Motion::create(si_);
        si_->copyState(motion->pState, st);
        mpStartTree->add(motion);
      }
      // check if there are no initial states
      if (mpStartTree->size() == 0)
      {
        LOG << "  " << getName().c_str() << " There are no valid initial states!" << std::endl;
        return false;
      }
      // add more potential states to the search graph #mpGoalTree
      while (const base::State *st = pis_.nextGoal())
      {
        Motion* motion = Motion::create(si_);
        si_->copyState(motion->pState, st);
        // reverse direction;
        motion->pState->as<MukStateType>()->rotation().x *= (-1);
        motion->pState->as<MukStateType>()->rotation().y *= (-1);
        motion->pState->as<MukStateType>()->rotation().z *= (-1);
        mpGoalTree->add(motion);
      }
      // check if there are no goal states
      if (mpGoalTree->size() == 0)
      {
        LOG << getName().c_str() << " There are no valid goal states!" << std::endl;
        return false;
      }
      return true;
    }

    /** \brief collects the results from the planning steps

    includes e.g. the building of a solution path, if one was found. Then adds it to the solutions paths in the problem definition.
    */
    void SplineBasedRRTconnect::postprocess(bool solved, bool approximated)
    {
      mStats.numberOfStatesCreated = mpStartTree->size() + mpGoalTree->size();
      // at least one valid state was found and connected to a initial state
      mpLastStartMotion = mConnectionPoint.first;
      mpLastGoalMotion  = mConnectionPoint.second;
      if (nullptr != mpLastStartMotion)
      {
        mStats.success = true;
        mStats.goalDist = 0.0;
        // construct the solution path
        std::vector<Motion*> pathFromStartTree; // begin with recursively iterating the StartTree
        while (nullptr != mpLastStartMotion)
        {
          pathFromStartTree.push_back(mpLastStartMotion);
          mpLastStartMotion = mpLastStartMotion->pParent;
        }
        // add the solution path to the problem definition
        auto omplPath = std::make_unique<PathGeometric>(si_);
        const auto N = pathFromStartTree.size();
        for (size_t i(0); i<pathFromStartTree.size(); ++i)
        {
          omplPath->append(pathFromStartTree[N-i-1]->pState);
        }
        Motion* lastMotion(nullptr);
        while (nullptr != mpLastGoalMotion)
        {
          omplPath->append(mpLastGoalMotion->pState);
          lastMotion = mpLastGoalMotion;
          mpLastGoalMotion = mpLastGoalMotion->pParent;
        }
        // do not know why i have to turn it again.
        auto state = gris::muk::fromOmplState(*lastMotion->pState->as<MukStateType>());
        state.tangent *= -1;
        gris::muk::setDirection(state, *lastMotion->pState->as<MukStateType>());
        mStats.goalAngle = 0; // dynamic_cast<base::MukGoalStates*>(pdef_->getGoal().get())->angleDistance(lastMotion->pState);
        pdef_->addSolutionPath(base::PathPtr(omplPath.release()), approximated, 0.0, getName());
      }
      else
      {
        mStats.success = false;
      }
    }

    /**
    */
    std::vector<Motion*> SplineBasedRRTconnect::selectNearestNeighbors(TreeType& tree, Motion* pMotion)
    {
      std::vector<Motion*> result;
      tree.nearestR(pMotion, mRadiusNearestBall, result);
      if (result.empty())
      {
        result.push_back(tree.nearest(pMotion));
      }
      return result;
    }

    /**
    */
    std::vector<Motion*> SplineBasedRRTconnect::selectStates(const std::vector<Motion*>& motions)
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
    std::vector<Motion*> SplineBasedRRTconnect::selectStatesFromOtherTree(TreeType& tree, Motion* motion)
    {
      std::vector<Motion*> candidates;
      tree.nearestR(motion, mRadiusNearestBall, candidates);
      if (candidates.empty())
      {
        candidates.push_back(tree.nearest(motion));
      }
      gris::Vec3d d_from;
      gris::muk::getDirection(*motion->pState->as<MukStateType>(), d_from);
      std::vector<Motion*> result;
      std::copy_if(candidates.begin(), candidates.end(), back_inserter(result), 
        [&] (Motion* from)
      {
        gris::Vec3d d_to;
        gris::muk::getDirection(*from->pState->as<MukStateType>(), d_to);
        bool inCone = insideCone(from, motion);
        auto dot = d_from.dot(d_to);
        return dot < 0 && inCone;//std::acos(dot) > mAngleConenctionThreshold;
      });
      return result;
    }
    
    /** \brief calculates a new state that fulfills the curvature constraint

    calculate the appropriate angle, to move under the curvature constraint from state "from" to state "to"
    due to the curv. constraint a minimum distance has to be calculated, this may be more than the maximum distance (*).

    reference paper: Yang et al., An analytical contiuous-curvature path-smoothing algorithm, IEEE, 2010 

    \param from:  the state, from which the new one is calculated
    \param to:    indicator, that tells the function in which direction the new state should lie, this state therefore will usually not be reached
    \param result: newly created state by this method, memory has to be allocated before
    \param distance: maximum distance to travel
    */
    Motion* SplineBasedRRTconnect::steer(TreeType& tree, Motion* from, const base::State* to, const base::PlannerTerminationCondition& ptc)
    {
      using namespace Eigen;
      double newDist = 0;
      double oldDist = std::numeric_limits<double>::infinity();
      Motion* parentMotion = from;

      //while (!ptc && oldDist > newDist)
      {
        // location and tangent at state "from"
        const auto* pStart = parentMotion->pState->as<MukStateType>();
        const auto* pGoal  = to->as<MukStateType>();
        Vector3d    dirStart = Vector3d(pStart->rotation().x, pStart->rotation().y, pStart->rotation().z);
        Vector3d    dirGoal  = Vector3d(pGoal->getX() - pStart->getX(), pGoal->getY() - pStart->getY(), pGoal->getZ() - pStart->getZ());
        dirStart.normalize();
        const double norm = dirGoal.norm();
        dirGoal /= norm;
        if (dirGoal.hasNaN())
          return nullptr;
        // angle between the two tangents
        const double dot = dirStart.dot(dirGoal);
        const double angle = acos(abs(dot));
        const double gamma = dot > 0 ?  angle : gris::M_PI - angle;
        const bool pointReachable = gamma < mSpiralBuilder.getGamma();
        double d = mSpiralBuilder.getMinLength();
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
          const auto steps = std::max(1u, static_cast<unsigned int>( angle / mSpiralBuilder.getGamma() + 0.5 ) );
          const double minApproxLength = steps * mStepSize;
          const double newAngle = 0.5 * std::min(1.0, norm / minApproxLength) *  mSpiralBuilder.getGamma();
          //d = mStepSize;
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
            return nullptr;
          }
          nextState->setXYZ( pStart->getX() + d*dirGoal.x(), pStart->getY() + d*dirGoal.y(), pStart->getZ() + d*dirGoal.z());
        }
        nextState->rotation().x = dirGoal.x();
        nextState->rotation().y = dirGoal.y();
        nextState->rotation().z = dirGoal.z();
        nextState->rotation().w = 0;
        if (motionIsValid(parentMotion->pState, nextMotion->pState))
        {
          tree.add(nextMotion);
          return nextMotion;
        }
        else
        {
          freeMotion(nextMotion);
          return nullptr;
        }
      }
    }

    /** \brief checks if a collision occurds between the state <pStart> and <pGoal>

      samples points along the line between pStart and pGoal and calls the collision detector for each point
    */
    bool SplineBasedRRTconnect::motionIsValid(const base::State* pStart, const base::State* pGoal) const
    {
      using gris::Vec3d;
      const double sampleRate = 0.25; // mm
      auto pWorkMotion = Motion::make_unique(si_);
      ompl::base::State* work = pWorkMotion->pState;
      si_->copyState(work, pStart);

      using T = MukStateSpace::StateType;

      auto* state = work->as<T>();
      auto* goal  = pGoal->as<T>();
      Vec3d tangent = Vec3d(goal->getX() - state->getX(),
        goal->getY() - state->getY(),
        goal->getZ() - state->getZ());
      const size_t N = static_cast<size_t>( ceil(tangent.norm() / sampleRate) );
      Vec3d step = (1.0 / N) * tangent;
      if (step.hasNaN())
        return false;
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

    /** \brief tries to connect the two trees along the two specified states
    */
    bool SplineBasedRRTconnect::connectTree(TreeType& treeFrom, Motion* motionFrom, TreeType& treeTo, Motion* motionTo, const base::PlannerTerminationCondition& ptc)
    {
      using namespace Eigen;
      using namespace gris::muk;

      double newDist = 0;
      double oldDist = std::numeric_limits<double>::infinity();
      
      while (!ptc && oldDist > newDist)
      {
        oldDist = newDist;
        {
          bool startToGoalValid = false;
          {
            // check validity from start to goal
            const auto* pStart = motionFrom->pState->as<MukStateType>();
            const auto* pGoal = motionTo->pState->as<MukStateType>();
            auto dirStart = Vector3d(pStart->rotation().x, pStart->rotation().y, pStart->rotation().z);
            auto dirStartToGoal = Vector3d(pGoal->getX() - pStart->getX(), pGoal->getY() - pStart->getY(), pGoal->getZ() - pStart->getZ());
            dirStart.normalize();
            const double norm = dirStartToGoal.norm();
            dirStartToGoal /= norm;
            if (!dirStartToGoal.hasNaN())
            {
              // angle between the two tangents
              const double dot = dirStart.dot(dirStartToGoal);
              const double angle = acos(abs(dot));
              const double gamma = dot > 0 ?  angle : gris::M_PI - angle;
              const bool pointReachable = gamma < mSpiralBuilder.getGamma();
              if (pointReachable && si_->distance(motionFrom->pState, motionTo->pState) < 3*mStepSize && motionIsValid(motionFrom->pState, motionTo->pState)) // point reachable
              {
                mConnectionPoint.first = motionFrom;
                mConnectionPoint.second = motionTo;
                startToGoalValid = true;
              }
            }
          }
          bool goalToStartValid = false;
          {
            // check validity from goal to start
            const auto* pGoal = motionFrom->pState->as<MukStateType>();
            const auto* pStart = motionTo->pState->as<MukStateType>();
            auto dirStart = Vector3d(pStart->rotation().x, pStart->rotation().y, pStart->rotation().z);
            auto dirStartToGoal = Vector3d(pGoal->getX() - pStart->getX(), pGoal->getY() - pStart->getY(), pGoal->getZ() - pStart->getZ());
            dirStart.normalize();
            const double norm = dirStartToGoal.norm();
            dirStartToGoal /= norm;
            if (!dirStartToGoal.hasNaN())
            {
              // angle between the two tangents
              const double dot = dirStart.dot(dirStartToGoal);
              const double angle = acos(abs(dot));
              const double gamma = dot > 0 ?  angle : gris::M_PI - angle;
              const bool pointReachable = gamma < mSpiralBuilder.getGamma();
              if (pointReachable && si_->distance(motionFrom->pState, motionTo->pState) < 3*mStepSize && motionIsValid(motionFrom->pState, motionTo->pState)) // point reachable
              {
                goalToStartValid = true;
              }
            }
          }
          if (startToGoalValid && goalToStartValid)
            return true;
        }

        auto* nextMotionFrom = steer(treeFrom, motionFrom, motionTo->pState, ptc);
        if (nextMotionFrom == nullptr)
        {
          return false;
        }
        auto* nextMotionTo = steer(treeTo, motionTo, nextMotionFrom->pState, ptc);
        if (nextMotionTo == nullptr)
        {
          return false;
        }
        newDist = si_->distance(nextMotionFrom->pState, nextMotionTo->pState);
        motionFrom = nextMotionFrom;
        motionTo = nextMotionTo;
      }
      return false;
    }

    /** \brief Check if the query point is within a cone of certain size

      A cone is simply an infinite number of circles whose size is defined by a linear equation that takes the distance from the point.
      Simply check if it's inside the circle of the appropriate size.
    */
    bool SplineBasedRRTconnect::insideCone(const Motion* src, const Motion* query) const
    {
      using namespace Eigen;
      const auto p       = positionAsEigen(query->pState);
      // determine cone parameters
      const auto coneTip = positionAsEigen(src->pState);
      const auto coneDir = directionAsEigen(src->pState);
      //const double r     = 2*mStepSize / 3.0;
      //const double h     = 2*mStepSize; // height
      // project onto center line.
      const double coneDist = (p-coneTip).dot(coneDir);
      //if (coneDist < 0 || coneDist > mConeLength)
      if (coneDist < 0)
        return false;
      const double coneRadius = (coneDist / mConeLength) * mConeRadius;
      const double orthDistance = ((p-coneTip) - coneDist*coneDir).norm();
      if (orthDistance < coneRadius || coneDist > mConeLength)
        return true;
      else
        return false;
    }

    /** \brief creates a Boost::Graph from the list of created motions
    */
    void SplineBasedRRTconnect::getPlannerData(base::PlannerData &data) const
    {
      Planner::getPlannerData(data);

      std::vector<Motion*> motions;
      if (mpStartTree)
        mpStartTree->list(motions);
      for (unsigned int i = 0; i < motions.size(); ++i)
      {
        if (nullptr == motions[i]->pParent)
          data.addStartVertex(base::PlannerDataVertex(motions[i]->pState));
        else
          data.addEdge(base::PlannerDataVertex(motions[i]->pParent->pState), base::PlannerDataVertex(motions[i]->pState));
      }

      if (mpGoalTree)
      {
        motions.clear();
        mpGoalTree->list(motions);
        for (unsigned int i = 0; i < motions.size(); ++i)
        {
          if (nullptr == motions[i]->pParent)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->pState));
          else
            data.addEdge(base::PlannerDataVertex(motions[i]->pParent->pState), base::PlannerDataVertex(motions[i]->pState));
        }
      }

      if (nullptr!=mpLastGoalMotion)
      {
        LOG_LINE << "building connection graphs";
        auto* pWork = mpLastGoalMotion;
        while (pWork)
        {
          if (nullptr == pWork->pParent)
            data.addStartVertex(base::PlannerDataVertex(pWork->pState));
          else
            data.addEdge(base::PlannerDataVertex(pWork->pParent->pState), base::PlannerDataVertex(pWork->pState));
          pWork = pWork->pParent;
        }
        pWork = mpLastStartMotion;
        while (pWork)
        {
          if (nullptr == pWork->pParent)
            data.addStartVertex(base::PlannerDataVertex(pWork->pState));
          else
            data.addEdge(base::PlannerDataVertex(pWork->pParent->pState), base::PlannerDataVertex(pWork->pState));
        }
      }
    }

    /**
    */
    size_t  SplineBasedRRTconnect::getNumberOfStartTreeRoots()   const
    {
      size_t counter(0);
      std::vector<Motion*> motions;
      if (mpStartTree)
        mpStartTree->list(motions);
      for (unsigned int i = 0; i < motions.size(); ++i)
      {
        if (nullptr == motions[i]->pParent)
          ++counter;
      }
      return counter;
    }

    /** see og::Planner::setup()
    */
    void SplineBasedRRTconnect::setup()
    {
      Planner::setup();

      if ( ! mpSampler)
      {
        mpSampler = si_->allocStateSampler();
      }

      if ( ! mpStartTree)
      {
        mpStartTree.reset(new NearestNeighborsSqrtApprox<Motion*>());
      }
      mpStartTree->setDistanceFunction([&] (const Motion *a, const Motion *b) { return this->distanceFunction(a,b); });

      if ( ! mpGoalTree)
      {
        mpGoalTree.reset(new NearestNeighborsSqrtApprox<Motion*>());
      }
      mpGoalTree->setDistanceFunction([&] (const Motion *a, const Motion *b) { return this->distanceFunction(a,b); });

      if ( ! mpConnectionAttempts)
      {
        mpConnectionAttempts.reset(new NearestNeighborsSqrtApprox<Motion*>());
      }
      mpConnectionAttempts->setDistanceFunction([&] (const Motion *a, const Motion *b) { return this->distanceFunction(a,b); });
    }

    /** see og::Planner::clear()
    */
    void SplineBasedRRTconnect::clear()
    {
      Planner::clear();
      mpSampler.reset();
      freeMemory();
      mpLastGoalMotion = nullptr;
      mpLastStartMotion = nullptr;
    }

    /**
    */
    void SplineBasedRRTconnect::restartSearch()
    {
      clear();
      //freeMemory();
      //mpLastGoalMotion = nullptr;
      //mpLastStartMotion = nullptr;
      //pis_.restart(); // planner input states
    }

    /** \briefs wraps distance function to distance function of the state space (SE3)
    */
    double SplineBasedRRTconnect::distanceFunction(const Motion *a, const Motion *b) const
    {
      //return si_->distance(a->pState, b->pState);
      auto* astate = a->pState->as<MukStateType>();
      auto* bstate = b->pState->as<MukStateType>();
      auto dist = std::pow(astate->getX()-bstate->getX(),2) + std::pow(astate->getY()-bstate->getY(),2) + std::pow(astate->getZ()-bstate->getZ(),2);
      return std::sqrt(dist);
    }

    /** \brief
      we cannot compute gamma analytically. Therefor we approximate it.
    */
    void SplineBasedRRTconnect::setStepSize(double val)
    {
      double minGamma = 0;
      double maxGamma = gris::M_PI;
      double eps      = 0.01;
      mSpiralBuilder.setGamma(maxGamma);
      approximateStepSize(eps, minGamma, maxGamma, val, mSpiralBuilder);
      mStepSize = mSpiralBuilder.getMinLength();
    }
  }
}

namespace
{
  /**
  */
  void approximateStepSize(double eps, double minGamma, double maxGamma, double stepSize, gris::muk::BezierSpiralInterpolator& spiralBuilder)
  {
    double length = spiralBuilder.getMinLength();
    double gamma  = spiralBuilder.getGamma();
    if ( abs(length-stepSize) > eps)
    {
      spiralBuilder.setGamma( 0.5*(minGamma + maxGamma) );
      if (length > stepSize)
      {
        maxGamma = gamma;
      }
      else
      {
        minGamma = gamma;
      }
      approximateStepSize(eps, minGamma, maxGamma, stepSize, spiralBuilder);
    }
  }
}