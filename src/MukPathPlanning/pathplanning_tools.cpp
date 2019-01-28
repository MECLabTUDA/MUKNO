#include "private/muk.pch"
#include "private/pathplanning_tools.h"
#include "private/types.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace
{
  using namespace gris::muk;
  void traverse_r(const ompl::base::PlannerData& data, unsigned int idx, MukNode& node);
}

namespace gris
{
  namespace muk
  {
    /** \brief Converts a MukState to an ompl-SE3-state where the xyz-component of the quaternion defines the direction

      old and .. bad implementation
    */
    ompl::base::ScopedState<> convert(const MukState& mukState, const ompl::base::StateSpacePtr& space)
    {
      ob::ScopedState<> ss(space);
      auto* state = ss->as<og::MukStateType>();
      setState(mukState, *state);

      if (0 == state->rotation().x &&
        0 == state->rotation().y &&
        0 == state->rotation().z)
      {
        state->rotation().x = 1;
      }
      space->enforceBounds(state);
      return ss;
    }

    /** \brief Converts a MukState to an ompl-SE3-state where the z-Axis is the direction
    */
    void toSE3(const MukState& mukState, og::MukStateType& state)
    {
      setPosition(mukState, state);

      const auto a = Eigen::Vector3d(0,0,1);
      const auto b = Eigen::Vector3d(mukState.tangent.x(), mukState.tangent.y(), mukState.tangent.z());
      auto  q = Eigen::Quaterniond::FromTwoVectors(a,b);
      auto& r = state.rotation();
      r.x = q.x();
      r.y = q.y();
      r.z = q.z();
      r.w = q.w();
    }

    /** \brief Converts a MukState to an ompl-SE3-state where the z-Axis is the direction
    */
    ompl::base::ScopedState<> toSE3(const MukState& mukState, const ompl::base::StateSpacePtr& space)
    {
      ob::ScopedState<> ss(space);
      auto* state = ss->as<og::MukStateType>();
      toSE3(mukState, *state);
      space->enforceBounds(state);
      return ss;
    }

    /**
    */
    MukState fromOmplState(const og::MukStateType& state)
    {
      return MukState(Vec3d(state.getX(), state.getY(), state.getZ())
        , Vec3d(state.rotation().x, state.rotation().y, state.rotation().z)
      );
    }

    /**
    */
    MukState fromSE3(const og::MukStateType& state)
    {
      const auto pos = Vec3d(state.getX(), state.getY(), state.getZ());
      Eigen::Quaterniond q;
      fromOmplState(state, q);
      auto dir = Eigen::Vector3d(0,0,1);
      dir = q.toRotationMatrix()*dir;
      dir.normalize();
      return MukState(pos, Vec3d(dir.data()));
    }

    /**
    */
    void fromOmplState(const og::MukStateType& state, Eigen::Quaterniond& q)
    {
      using namespace ompl::geometric;
      const auto& r = state.rotation();
      q.x() = r.x;
      q.y() = r.y;
      q.z() = r.z;
      q.w() = r.w;
    }

    /**
    */
    void fromOmplState(const og::MukStateType& state, Eigen::Vector3d& p)
    {
      using namespace ompl::geometric;
      p.x() = state.getX();
      p.y() = state.getY();
      p.z() = state.getZ();
    }

    /**
    */
    void fromOmplState(const og::MukStateType& state, Eigen::Matrix4d& M)
    {
      using namespace ompl::geometric;
      const auto& r = state.rotation();
      // translation
      M.setIdentity();
      M(0,3) = state.getX();
      M(1,3) = state.getY();
      M(2,3) = state.getZ();
      // rotation
      Eigen::Quaterniond q;
      q.x() = r.x;
      q.y() = r.y;
      q.z() = r.z;
      q.w() = r.w;
      auto R = Eigen::AngleAxisd(q).toRotationMatrix();
      for (size_t i(0); i<3; ++i)
        for (size_t j(0); j<3; ++j)
          M(i,j) = R(i,j);
    }

    /**
    */
    void setState(const MukState& state, og::MukStateType& dest)
    {
      setPosition(state, dest);
      setDirection(state, dest);
    }

    /**
    */
    void setPosition(const MukState& state, og::MukStateType& dest)
    {
      dest.setXYZ(state.coords.x(), state.coords.y(), state.coords.z());
    }

    /**
    */
    void setDirection(const MukState& state, og::MukStateType& dest)
    {
      dest.rotation().x = state.tangent.x();
      dest.rotation().y = state.tangent.y();
      dest.rotation().z = state.tangent.z();
      dest.rotation().w = 0;
    }

    /**
    */
    void getState(const og::MukStateType& state, MukState& dest)
    {
      getPosition(state, dest.coords);
      getDirection(state, dest.tangent);
    }

    /**
    */
    void getPosition(const og::MukStateType& state, Vec3d& dest)
    {
      dest.x() = state.getX();
      dest.y() = state.getY();
      dest.z() = state.getZ();
    }

    /**
    */
    void getDirection(const og::MukStateType& state, Vec3d& dest)
    {
      dest.x() = state.rotation().x;
      dest.y() = state.rotation().y;
      dest.z() = state.rotation().z;
    }

    /**
    */
    MukPathGraph extractRoadmap(const ob::PlannerData& plannerData)
    {
      MukPathGraph graph;
      for(unsigned int i(0); i<plannerData.numStartVertices(); ++i)
      {
        MukNode node;
        traverse_r(plannerData, plannerData.getStartIndex(i), node);
        graph.addRoot(std::move(node));
      }
      for(unsigned int i(0); i<plannerData.numGoalVertices(); ++i)
      {
        MukNode node;
        traverse_r(plannerData, plannerData.getGoalIndex(i), node);
        graph.addRoot(std::move(node));
      }
      return graph;
    }

    /** \brief Rearranges the states so they go from back to front.

      "Start -> -> -> -> Goal" becomes "Goal -> -> -> -> Start"
    */
    MukPath swapDirection(const MukPath& input)
    {
      MukPath result = input;
      auto& in  = input.getPath();
      auto& out = result.getPath();
      const auto N = out.size();
      for (size_t i(0); i<N; ++i)
      {
        out[i] = in[N - 1 - i];
        out[i].tangent *= -1;
      }
      return result;
    }
  }
}

namespace
{
  void traverse_r(const ompl::base::PlannerData& data, unsigned int idx, MukNode& node)
  {
    const ompl::base::PlannerDataVertex& v = data.getVertex(idx);
    node.mState = fromOmplState(*v.getState()->as<og::MukStateType>());

    std::vector<unsigned int> edgeList;
    data.getEdges(idx, edgeList);
    node.children.resize(edgeList.size());
    int i(0);
    for (auto idxNew : edgeList)
    {
      traverse_r(data, idxNew, node.children[i]);
      ++i;
    }
  }
}