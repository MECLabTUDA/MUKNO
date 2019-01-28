#pragma once
#include "private/types.h"

#include "MukCommon/MukState.h"
#include "MukCommon/MukPath.h"
#include "MukCommon/MukPathGraph.h"

#pragma warning (push)
#pragma warning( disable: 4267 ) // ompl -> conversion from size_t to unsigned int 
#pragma warning( disable: 4800 ) // forcing value to bool
#pragma warning( disable: 4996 ) // function (localtime) may be unsafe
#include <ompl/base/PlannerData.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#pragma warning (pop)

#include <Eigen/Dense>

namespace gris
{
  namespace muk
  {
    ompl::base::ScopedState<> convert(const MukState& mukState, const ompl::base::StateSpacePtr& space);
    ompl::base::ScopedState<> toSE3  (const MukState& mukState, const ompl::base::StateSpacePtr& space);
    void toSE3(const MukState& mukState, og::MukStateType& state);

    MukState                  fromSE3(const og::MukStateType& state);

    MukState                 fromOmplState(const og::MukStateType& state);

    void fromOmplState(const og::MukStateType& state, Eigen::Quaterniond& q);
    void fromOmplState(const og::MukStateType& state, Eigen::Vector3d& p);
    void fromOmplState(const og::MukStateType& state, Eigen::Matrix4d& M);
    
    void setState    (const MukState& state, og::MukStateType& dest);
    void setPosition (const MukState& state, og::MukStateType& dest);
    void setDirection(const MukState& state, og::MukStateType& dest);

    void getState    (const og::MukStateType& state, MukState& dest);
    void getPosition (const og::MukStateType& state, Vec3d& dest);
    void getDirection(const og::MukStateType& state, Vec3d& dest);

    MukPathGraph extractRoadmap(const ob::PlannerData& plannerData);

    MukPath swapDirection(const MukPath& input);
  }
}