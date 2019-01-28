#include "private/muk.pch"
#include "VoxelCoordinate.h"

#include <itkImage.h>

namespace
{
  template<typename S, typename T>
  void fill(const S& src, T& dest)
  {
    for(size_t i(0); i<3; ++i)
      dest[i] = src[i];
  }
}

namespace gris
{
namespace muk
{
  /**
  */
  VoxelCoordinate::VoxelCoordinate(const ImageInt3D& image)
    : mImg(image)
  {
    /*auto spacing = image.GetSpacing();
    for (int i(0); i<3; ++i)
      mSpacing[i] = spacing[i];
    auto size = image.GetLargestPossibleRegion().GetSize();
    for (int i(0); i<3; ++i)
      mDimension[i] = size[i];*/
  }

  /**
  */
  VoxelCoordinate::VoxelCoordinate(const ImageInt3D& image, const ImageInt3DIndex& index)
    : mIndex(index)
    , mImg(image)
  {
    /*auto spacing = image.GetSpacing();
    for (int i(0); i<3; ++i)
      mSpacing[i] = spacing[i];
    auto size = image.GetLargestPossibleRegion().GetSize();
    for (int i(0); i<3; ++i)
      mDimension[i] = size[i];*/
  }

  /**
  */
  void VoxelCoordinate::setIndex(const ImageInt3DIndex& index)
  {
    mIndex = index;
  }

  /**
  */
  void VoxelCoordinate::setFromWorldCoordinate(const Vec3d& p)
  {
    /*for(size_t i(0); i<3; ++i)
      mIndex[i] = static_cast<int>(std::round(p[i] / mSpacing[i]));*/
    ItkPoint q;
    fill(p, q);
    ItkImageIndex index;
    mImg.TransformPhysicalPointToIndex(q, index);
    fill(index, mIndex);
  }

  /**
  */
  Vec3d VoxelCoordinate::toWorldCoordinate() const
  {
    ItkImageIndex index;
    fill(mIndex, index);
    ItkPoint q;
    mImg.TransformIndexToPhysicalPoint(index, q);
    Vec3d res;
    fill(q, res);
    return res;
  }

  /**
  */
  ImageInt3DIndex VoxelCoordinate::clamped(const Vec3d& p) const
  {
    ItkPoint q;
    fill(p, q);
    ItkImageIndex index;
    mImg.TransformPhysicalPointToIndex(q, index);
    auto maxRegion = mImg.GetLargestPossibleRegion();
    auto maxIndex = maxRegion.GetUpperIndex();
    {
      ItkPoint q;
      mImg.TransformIndexToPhysicalPoint(maxIndex, q);
    }

    for (size_t i(0); i<3; ++i)
    {
      index[i] = std::max((ItkImageIndex::IndexValueType)0, std::min(index[i], maxIndex[i]));
    }
    ImageInt3DIndex res;
    fill(index, res);
    return res;
  }
}
}

/** \brief Converts the  @worldCoords with bounds @mpBounds to @sliceCoords in a 3d volume with extent @extent
*/
//void CtTransformsLogic::convertWorldToSliceCoordinates(const Vec3d& worldCoords, Vec3i& sliceCoords, int* extent)
//{
//  double sliceThicknessX = extentX / static_cast<double>(extent[0] + extent[1]);
//  double sliceThicknessY = extentY / static_cast<double>(extent[2] + extent[3]);
//  double sliceThicknessZ = extentZ / static_cast<double>(extent[4] + extent[5]);
//
//  double extentSliceX = worldCoords.x;
//  double extentSliceY = worldCoords.y;
//  double extentSliceZ = worldCoords.z;
//
//  double sliceNumberX = extentSliceX / sliceThicknessX;
//  double sliceNumberY = extentSliceY / sliceThicknessY;
//  double sliceNumberZ = extentSliceZ / sliceThicknessZ;
//
//  sliceCoords[0] = std::round(sliceNumberX);
//  sliceCoords[1] = std::round(sliceNumberY);
//  sliceCoords[2] = std::round(sliceNumberZ);
//}