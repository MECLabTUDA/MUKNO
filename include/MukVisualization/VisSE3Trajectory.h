#pragma once
#include "VisAbstractObject.h"

#include "MukCommon/LieGroupSE3.h"

namespace gris
{
  namespace muk
  {
    /** \brief Visualization of a series of poses
    */
    class MUK_VIS_API VisSE3Trajectory : public VisAbstractObject
    {
      public:
        explicit VisSE3Trajectory(const std::string& name);
        ~VisSE3Trajectory();

      public:
        virtual void update();
        void setNumberOfStates(size_t n);
        void setPose(size_t i, const se3::Pose3D& pose);
        void setPoses(const std::vector<se3::Pose3D>& poses);
      
      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}