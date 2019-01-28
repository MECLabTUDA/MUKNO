#include "private/muk.pch"
#include "statistics.h"
#include "private/muk.pch"
#include "statistics.h"

#include "MukCommon/MukState.h"
#include "MukCommon/ICollisionDetector.h"
#include "MukCommon/geometry.h"

#include <numeric>

namespace gris
{
  namespace muk
  {
    /**
    */
    void Statistics1D::compute()
    {
      std::sort(mData.begin(), mData.end());
      if (mData.empty())
      {
        mMin          = 0;
        mLowerQuartil = 0;
        mMedian       = 0;
        mUpperQuartil = 0;
        mMax          = 0;
        mMean         = 0;
        mStdev        = 0;
      }
      else
      {
        mMin = mData.front();
        mMax = mData.back();
        mMedian       = mData[std::min(mData.size()-1, (size_t)std::round(0.5*mData.size()))];
        mLowerQuartil = mData[std::min(mData.size()-1, (size_t)std::round(0.25*mData.size()))];
        mUpperQuartil = mData[std::min(mData.size()-1, (size_t)std::round(0.75*mData.size()))];
        mMean         = std::accumulate(mData.begin(), mData.end(), 0.0) / mData.size();
        std::vector<double> diff(mData.size());
        std::transform(mData.begin(), mData.end(), diff.begin(), [&] (double x) { return x - mMean; });
        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        mStdev = std::sqrt(sq_sum / mData.size());
      }
    }

    /**
    */
    void BoxPlotData::compute()
    {
      std::sort(mData.begin(), mData.end());
      if (mData.empty())
      {
        mMin          = 0;
        mLowerQuartil = 0;
        mMedian       = 0;
        mUpperQuartil = 0;
        mMax          = 0;
      }
      else
      {
        mMin = mData.front();
        mMax = mData.back();
        mMedian       = mData[std::min(mData.size()-1, (size_t)std::round(0.5*mData.size()))];
        mLowerQuartil = mData[std::min(mData.size()-1, (size_t)std::round(0.25*mData.size()))];
        mUpperQuartil = mData[std::min(mData.size()-1, (size_t)std::round(0.75*mData.size()))];
      }
    }

    /**
    */
    std::vector<double> computeDistances(ICollisionDetector& detector, const std::vector<MukState>& data, double radius)
    {   
      std::vector<double> result;
      Vec3d nn;
      std::for_each(data.begin(), data.end(), [&] (const MukState& state)
      {
        //const size_t SampleSize = 72;
        //std::vector<Vec3d> samples(SampleSize); // all 5 degrees
        //circleAroundLine(state.coords, state.tangent, radius, samples);
        //std::vector<double> tmpResult(SampleSize, std::numeric_limits<double>::max());
        /*std::for_each(samples.begin(), samples.end(), [&] (const Vec3d& p)
          {
            detector.nearestNeighbor(p, nn);
            if (nn.hasNaN())
              tmpResult.push_back(std::numeric_limits<double>::max());
            else
            {
              const double val = (state.coords-nn).squaredNorm();
              if (val < radius*radius)
                tmpResult.push_back( - std::sqrt(val));
              else
                tmpResult.push_back((p-nn).norm());
            }
          });
        result.push_back(*std::min_element(tmpResult.begin(), tmpResult.end()));*/

        detector.nearestNeighbor(state.coords, nn);
        if (nn.hasNaN())
          result.push_back(std::numeric_limits<double>::max());
        else
          result.push_back((state.coords-nn).norm()-radius);
      });
      return result;
    }

    /**
    */    
    std::vector<double> computeCurvatures(const std::vector<MukState>& data)
    {
      std::vector<double> curvatures;
        
      if (data.empty())
        return curvatures;
      curvatures.push_back(0);
      if (data.size()>2)
      {
        for (size_t i(1); i<data.size()-2; ++i)
        {
          Sphere3D sphere;
          fitSphere(data[i-1].coords, data[i].coords, data[i+1].coords, sphere);
          const double r = sphere.getRadius();
          if (r==std::numeric_limits<double>::infinity())
            curvatures.push_back(0);
          else
            curvatures.push_back(1 / sphere.getRadius());
        }
      }
      curvatures.push_back(0);
      return curvatures;
    }

    /**
    */
    double computeLength(const std::vector<MukState>& data)
    {
      const size_t N = data.size();
      double length (0);
      for (size_t i(1); i<N; ++i)
      {
        length += (data[i-1].coords - data[i].coords).norm();
      }
      return length;
    }
  }
}
