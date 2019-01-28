#pragma once

#include "muk_imaging_api.h"
#include "MukImage.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_IMG_API VoxelCoordinate
    {
      public:
        explicit VoxelCoordinate(const ImageInt3D& im);
        explicit VoxelCoordinate(const ImageInt3D& im, const ImageInt3DIndex&);

      public:
        const ImageInt3DIndex& getIndex() const { return mIndex; }

      public:
        void setIndex(const ImageInt3DIndex& index);
        void setFromWorldCoordinate(const Vec3d& p);

      public:
        Vec3d         toWorldCoordinate()     const;
        ImageInt3DIndex clamped(const Vec3d& p) const;
        

      private:
        ImageInt3DIndex mIndex;

        const ImageInt3D& mImg;
        /*Vec3i mDimension;
        Vec3d mSpacing;*/
    };
  }
}
