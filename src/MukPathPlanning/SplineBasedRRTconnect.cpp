#include "private\muk.pch"
#include "private/SplineBasedRRTconnect.h"

#include "private/IStateValidityChecker.h"
#include "private/magic_numbers.h"
#include "private/MotionValidatorDefault.h"
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
  bool steering;

  // ----
  gris::Vec3d positionAsVec3d(const ompl::base::State* input)
  {
    auto* state = input->as<ompl::geometric::MukStateType>();
    return gris::Vec3d(state->getX(), state->getY(), state->getZ());
  }
  gris::Vec3d directionAsVec3d(const ompl::base::State* input)
  {
    auto* state = input->as<ompl::geometric::MukStateType>();
    return gris::Vec3d(state->rotation().x, state->rotation().y, state->rotation().z).normalized();
  }
  // ----
  gris::Vec3d positionAsVec3d(const ompl::geometric::Motion* motion)
  {
    return positionAsVec3d(motion->pState);
  }
  gris::Vec3d directionAsVec3d(const ompl::geometric::Motion* motion)
  {
    return directionAsVec3d(motion->pState);
  }
  // ----
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
    , mVerbose(false)
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
    auto* goal          = pdef_->getGoal().get();
    auto* goal_s        = dynamic_cast<base::GoalSampleableRegion*>(goal);
    bool  disconnected  = true;
    auto  pSample       = Motion::make_unique(si_);
    auto* rstate        = pSample->pState;
    auto  mUseStartTree = false;
    mConnectionPoint    = std::make_pair(nullptr, nullptr);
    mpLastGoalMotion    = nullptr;
    mpLastStartMotion   = nullptr;
    
    // try initial connection
    {
      /*if (connectTree(treeFrom, newMotions[i], treeTo, other, ptc))
      {
        disconnected = false;
        if ( ! mUseStartTree)
          std::swap(mConnectionPoint.first, mConnectionPoint.second);
        break;
      }*/
    }

    // go for it
    while (ptc == false && disconnected)
    {
      mUseStartTree = !mUseStartTree;
      TreeType& treeFrom = mUseStartTree ? *mpStartTree : *mpGoalTree;
      TreeType& treeTo   = mUseStartTree ? *mpGoalTree  : *mpStartTree;
        
      mpSampler->sampleUniform(rstate);

      auto nearestBall   = selectNearestNeighbors(treeFrom, pSample.get());
      auto nearestStates = selectStates(nearestBall);
      
      // usual steering method of an RRT
      steering = true;
      std::vector<Motion*> newMotions;
      for (auto* pNearest : nearestStates)
      {
        // create new states in #treeFrom
        auto motion = steer(treeFrom, pNearest, rstate, ptc);
        if(nullptr != motion)
          newMotions.push_back(motion);
      }
      steering = false;
      // connection attempt of an RRT-connect
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
    if (mVerbose)
    {
      LOG << "  " << name_ << ": created " << mpStartTree->size() + mpGoalTree->size() << " states in " << diff << " milliseconds.\n"
        << "    Created " << mpStartTree->size() << " states from initial states.\n"
        << "    Created " << mpGoalTree->size()  << " states from goal states.\n" ;
      LOG_LINE;
    }
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

    mSpiralBuilder.setKappa(mKappa);
    mSpiralBuilder.setMinLength(0.5*mStepSize);
    const auto gamma = mSpiralBuilder.fromKappaAndMinLength(mKappa, 0.5*mStepSize);
    mSpiralBuilder.setGamma(gamma);

    // add more potential states to the search graph #mpStartTree
    while (const base::State *st = pis_.nextStart())
    {
      Motion* motion = Motion::create(si_);
      motion->isRoot = true;
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
      motion->isRoot = true;
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
        auto state = gris::muk::fromOmplState(*mpLastGoalMotion->pState->as<MukStateType>());
        state.tangent *= -1;
        gris::muk::setDirection(state, *mpLastGoalMotion->pState->as<MukStateType>());
        omplPath->append(mpLastGoalMotion->pState);
        lastMotion = mpLastGoalMotion;
        mpLastGoalMotion = mpLastGoalMotion->pParent;
      }
      mStats.goalAngle = 0;
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
      /*auto* motion = tree.nearest(pMotion);
      tree.nearestR(motion, mRadiusNearestBall, result);*/
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
    result.erase(std::remove_if(result.begin(), result.end(), [&](const Motion* motion) { return motion->numChildren > mMaxChildren; }), result.end());
    std::unique(result.begin(), result.end(), [&](auto* lhs, auto* rhs) { return lhs == rhs; });
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
    /*if (result.size() > 0)
      LOG_LINE << "found " << result.size() << " candidates";*/
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
    {
      // location and tangent at state "from"
      const auto* pStart   = parentMotion->pState->as<MukStateType>();
      const auto* pGoal    = to->as<MukStateType>();
      auto        dirStart = Vector3d(pStart->rotation().x, pStart->rotation().y, pStart->rotation().z);
      auto        dirGoal  = Vector3d(pGoal->getX() - pStart->getX(), pGoal->getY() - pStart->getY(), pGoal->getZ() - pStart->getZ());
      dirStart.normalize();
      const double norm = dirGoal.norm();
      dirGoal /= norm;
      if (dirGoal.hasNaN())
        return nullptr;
      // angle between the two tangents
      const double dot   = dirStart.dot(dirGoal);
      const double angle = acos(abs(dot));
      const double gamma = dot > 0 ?  angle : gris::M_Pi - angle;
      const bool pointReachable = gamma < mSpiralBuilder.getGamma();
      double d = mSpiralBuilder.getMinLength();
      // new motion
      auto* nextMotion = Motion::create(si_);
      nextMotion->pParent = parentMotion;
      auto* nextState = nextMotion->pState->as<MukStateType>();
      if (from->isRoot)
      {
        nextState->setXYZ( pStart->getX() + mStepSize*dirStart.x(), pStart->getY() + mStepSize*dirStart.y(), pStart->getZ() + mStepSize*dirStart.z());
        dirGoal = dirStart;
      }
      else if (pointReachable)
      {
        nextState->setXYZ( pStart->getX() + mStepSize*dirGoal.x(), pStart->getY() + mStepSize*dirGoal.y(), pStart->getZ() + mStepSize*dirGoal.z());
      }
      else
      {
        auto* grandParent = parentMotion->pParent;
        auto* gpState     = grandParent->pState->as<MukStateType>();
        const auto w1 = Vector3d(gpState->getX(), gpState->getY(), gpState->getZ());
        const auto w2 = Vector3d(pStart->getX(), pStart->getY(), pStart->getZ());
        const auto w3 = Vector3d(pGoal->getX(), pGoal->getY(), pGoal->getZ());
        const auto tB = (w2 - w1).normalized();
        const auto tE = (w3 - w2).normalized();
        const auto n = tB.cross(tE);
        const auto R = AngleAxisd(mSpiralBuilder.getGamma(), n);
        const auto tE_new = R*tB;
        const auto wNew = Vector3d(w2 + mStepSize*tE_new);
        nextState->setXYZ( wNew.x(), wNew.y(), wNew.z() );
        dirGoal = tE_new;
      }
      nextState->rotation().x = dirGoal.x();
      nextState->rotation().y = dirGoal.y();
      nextState->rotation().z = dirGoal.z();
      nextState->rotation().w = 0;
      if (motionIsValid(parentMotion->pState, nextMotion->pState))
      {
        ++nextMotion->pParent->numChildren;
        tree.add(nextMotion);
        auto dist = [&] (base::State* a, base::State* b)
        {
          auto* pA = a->as<MukStateType>();
          auto* pB = b->as<MukStateType>();
          auto dx = pA->getX() - pB->getX();
          auto dy = pA->getY() - pB->getY();
          auto dz = pA->getZ() - pB->getZ();
          return std::sqrt(dx*dx+dy*dy+dz*dz);
        };
        /*if (dist(from->pState, nextMotion->pState) < 0.9*mStepSize)
          LOG << "   dist: (" << steering << ") "<< dist(from->pState, nextMotion->pState) << "\n";*/
        return nextMotion;
      }
      else
      {
        freeMotion(nextMotion);
        return nullptr;
      }
    }
  }

  /** \brief checks if a collision occurs between the state <pStart> and <pGoal>

    samples points along the line between pStart and pGoal and calls the collision detector for each point
  */
  bool SplineBasedRRTconnect::motionIsValid(const base::State* pStart, const base::State* pGoal) const
  {
    using gris::Vec3d;
    auto pWorkMotion = Motion::make_unique(si_);
    ompl::base::State* work = pWorkMotion->pState;
    si_->copyState(work, pStart);

    using T = MukStateSpace::StateType;

    auto* state = work->as<T>();
    auto* goal  = pGoal->as<T>();
    Vec3d line  = Vec3d(goal->getX() - state->getX(),
      goal->getY() - state->getY(),
      goal->getZ() - state->getZ());
    const size_t N = static_cast<size_t>( ceil(line.norm() / gris::planning::Magic_Collision_Query_Resolution) );
    Vec3d step = (1.0 / N) * line;
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

  /** \brief Tries to connect both trees by moving from #motionFrom to #motionTo

    If the distance between the two last motions is larger than 3* #mStepSize, an additional motion is added.
    If both last states are close enough, it checks for feasibility of the two splines that would connect the trees.
  */
  bool SplineBasedRRTconnect::connectTree(TreeType& treeFrom, Motion* motionFrom, TreeType& treeTo, Motion* motionTo, const base::PlannerTerminationCondition& ptc)
  {
    using namespace Eigen;
    using namespace gris::muk;
    double newDist = std::numeric_limits<double>::max();
    double oldDist = std::numeric_limits<double>::infinity();
    bool   extendTreeFrom = true;
    while (!ptc && oldDist > newDist)
    {
      oldDist = newDist;
      newDist = si_->distance(motionFrom->pState, motionTo->pState);
      auto dist = [&] (base::State* a, base::State* b)
      {
        auto* pA = a->as<MukStateType>();
        auto* pB = b->as<MukStateType>();
        auto dx = pA->getX() - pB->getX();
        auto dy = pA->getY() - pB->getY();
        auto dz = pA->getZ() - pB->getZ();
        return std::sqrt(dx*dx+dy*dy+dz*dz);
      };
      if (dist(motionFrom->pState, motionTo->pState) > 2*mStepSize)
      {
        // the two trees are now too far away, make on small step and try again
        Motion* nextMotion(nullptr);
        if (extendTreeFrom)
        {
          nextMotion = steer(treeFrom, motionFrom, motionTo->pState, ptc);
          motionFrom = nextMotion;
        }
        else
        {
          nextMotion = steer(treeTo, motionTo, motionFrom->pState, ptc);
          motionTo   = nextMotion;
        }
        if (nextMotion == nullptr)
        {
          return false;
        }
        extendTreeFrom = ! extendTreeFrom;
      }
      else
      {
        // states are definitely close
        // check if they are too close
        if (dist(motionFrom->pState, motionTo->pState) < 0.5*mStepSize)
        {
          if (motionFrom->pParent)
            motionFrom = motionFrom->pParent;
          else if (motionTo->pParent)
            motionTo = motionTo->pParent;
          else
            return false;
        }
        // the two trees are in range. If the angle constraint is satisfied, we can consider the two trees connected.
        bool startToGoalValid = false;
        {
          // check validity from start to goal
          Vector3d dirStart, dirGoal;
          directionsFromMotions(motionFrom, motionTo, dirStart, dirGoal);
          if (!dirGoal.hasNaN())
          {
            // angle between the two tangents
            if ( curvatureSatisfied(gris::Vec3d(dirStart.data()), gris::Vec3d(dirGoal.data()))
              &&  motionIsValid(motionFrom->pState, motionTo->pState))
            {
              mConnectionPoint.first  = motionFrom;
              mConnectionPoint.second = motionTo;
              startToGoalValid = true;
            }
          }
        }
        bool goalToStartValid = false;
        {
          // check validity from goal to start
          Vector3d dirStart, dirGoal;
          directionsFromMotions(motionTo, motionFrom, dirStart, dirGoal);
          if (!dirGoal.hasNaN())
          {
            // angle between the two tangents
            if (  curvatureSatisfied(gris::Vec3d(dirStart.data()), gris::Vec3d(dirGoal.data()))
              && motionIsValid(motionTo->pState, motionFrom->pState))
            {
              goalToStartValid = true;
            }
          }
        }
        if (startToGoalValid && goalToStartValid)
          return true;
      }
    }
    return false;
  }

  /** \brief Check if the query point is within a cone of certain size

    A cone is simply an infinite number of circles whose size is defined by a linear equation that takes the distance from the point.
    Simply check if it's inside the circle of the appropriate size.
  */
  bool SplineBasedRRTconnect::insideCone(const Motion* src, const Motion* query) const
  {
    const auto q = positionAsVec3d(query->pState);
    const auto p = positionAsVec3d(src->pState);
    const auto v = directionAsVec3d(src->pState);
    gris::muk::Cone3D cone;
    cone.setConeTip(p);
    cone.setConeDirection(v);
    cone.setRadius(mConeRadius);
    cone.setLength(mConeLength);
    return cone.isInside(q);
  }

  /** \brief Checks if the angle between v and w is smaller than gamma
    
    \param v a normalized vector
    \param w a normalized vector
  */
  bool SplineBasedRRTconnect::curvatureSatisfied(const gris::Vec3d& v, const gris::Vec3d& w) const
  {
    const double dot   = v.dot(w);
    const double angle = acos(abs(dot));
    const double gamma = dot > 0 ?  angle : gris::M_Pi - angle;
    return gamma < mSpiralBuilder.getGamma();
  }

  /** \brief Checks if the angle between v and w is smaller than gamma

    \param v a normalized vector
    \param w a normalized vector
  */
  /*bool SplineBasedRRTconnect::curvatureSatisfied(const Eigen::Vector3d& v, const Eigen::Vector3d& w) const
  {
    const double dot   = v.dot(w);
    const double angle = acos(abs(dot));
    const double gamma = dot > 0 ?  angle : gris::M_Pi - angle;
    return gamma < mSpiralBuilder.getGamma();
  }*/

  /** \brief Extracts two normalized direction vectors #vFrom and #vTo
  */
  void SplineBasedRRTconnect::directionsFromMotions(const Motion* mFrom, const Motion* mTo,  Eigen::Vector3d& vFrom, Eigen::Vector3d& vTo) const
  {
    using namespace Eigen;
    const auto* pGoal   = mFrom->pState->as<MukStateType>();
    const auto* pStart  = mTo->pState->as<MukStateType>();
    vFrom       = Vector3d(pStart->rotation().x, pStart->rotation().y, pStart->rotation().z).normalized();
    vTo         = Vector3d(pGoal->getX() - pStart->getX(), pGoal->getY() - pStart->getY(), pGoal->getZ() - pStart->getZ()).normalized();
  }

  /** \brief creates a boost::Graph from the list of created motions
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
  */
  void SplineBasedRRTconnect::setStepSize(double val)
  {
    mStepSize = val;
  }
}
}