#pragma once
#include "LieGroupSE3.h"

namespace gris
{
  namespace se3
  {
    /** \brief A pose (position and orientation) of the special Euclidean group SE(3) as homogenous matrix
    */
    class MUK_COMMON_API PoseMatrix
    {
      public:
        PoseMatrix() { mData.setIdentity(); }
        explicit PoseMatrix(Matrix4d&& m) : mData(std::move(m)) {}

      public:
        const Matrix4d& data() const { return mData; }
        TwistMatrix     log() const;

      public:
        PoseMatrix    operator*(const PoseMatrix& o) const;
        const double& operator()(int i, int j) const { return mData(i,j); }
        double&       operator()(int i, int j)       { return mData(i,j); }

      public:
        void  setPosition   (const Vec3d& p);
        void  setOrientation(const Quaternion& p);
        Vec3d getPosition() const;

      private:
        Matrix4d mData;
    };

    using Pose3D = PoseMatrix;
  }
}