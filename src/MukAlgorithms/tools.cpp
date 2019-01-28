#include "private/muk.pch"
#include "private/tools.h"

#include <itkImageRegionConstIterator.h>

namespace gris
{
namespace muk
{
  /** \brief extracts the indices of each regions

    The input parameter #v has to have the size of the number of objects in the labeled image!

   \param v  the vector to be filled with the indices
   \param pimg pointer to the labeled image
  */
  void extractRegionsFromLabelImage(std::vector<std::vector<ImageInt3D::IndexType>>& v, const ImageInt3D* pImg)
  {
    // fill with indices
    auto iter = itk::ImageRegionConstIterator<ImageInt3D>(pImg, pImg->GetBufferedRegion());
    while (!iter.IsAtEnd())
    {
      auto val = iter.Get();
      if (val)
      {
        v[val-1].push_back(iter.GetIndex());
      }
      ++iter;
    }
  }

  /**
  */
  void minimize(const ImageInt3D::IndexType& in, ImageInt3D::IndexType& out)
  {
    for (int i(0); i<3; ++i)
    {
      if (in[i] < out[i])
        out[i] = in[i];
    }
  }

  /**
  */
  void maximize(const ImageInt3D::IndexType& in, ImageInt3D::IndexType& out)
  {
    for (int i(0); i<3; ++i)
    {
      if (in[i] > out[i])
        out[i] = in[i];
    }
  }
}
}