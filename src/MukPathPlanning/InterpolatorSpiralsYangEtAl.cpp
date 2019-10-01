#include "private/muk.pch"
#include "BezierSpiralInterpolator.h"

#include "InterpolatorSpiralsYangEtAl.h"
#include "InterpolatorLinear.h"

#include "MukCommon/MukException.h"
#include "MukCommon/InterpolatorFactory.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace
{
  using gris::Vec3d;
  using namespace gris::muk;

  MukPath computeLinearly(const Vec3d& start, const Vec3d& end, double resolution);
  MukPath computeLinearly(const Vec3d& start, const Vec3d& end, size_t discretization);
}


namespace gris
{
  namespace muk
  {
    REGISTER_INTERPOLATOR(InterpolatorSpiralsYangEtAl);

    /**
    */
    struct InterpolatorSpiralsYangEtAl::Impl
    {
      explicit Impl(InterpolatorSpiralsYangEtAl* parent);
      ~Impl() {}

      void computeSamples       (std::vector<Vec3d>& v);
      void computeDiscretization(std::vector<Vec3d>& v, size_t mSamples);
      void computeResolution    (std::vector<Vec3d>& v, double resolution);
      

      InterpolatorSpiralsYangEtAl*  mParent;
      BezierSpiralInterpolator      interpolator;
      std::vector<BezierSpline>     splines;
      std::vector<bool>             splinesFlagsValid; // best to compute this along with the interpolation. only used for coloring/determine which points of the resulting mukpath are valid
      std::vector<bool>             mValidSplinePoints;
    };

    InterpolatorSpiralsYangEtAl::Impl::Impl(InterpolatorSpiralsYangEtAl* parent_)
      : mParent(parent_)
    {
      interpolator.setKappa(0);
    }


    // ----------------------------------------------------------------------------------------

    /**
    */
    InterpolatorSpiralsYangEtAl::InterpolatorSpiralsYangEtAl()
      : IInterpolator()
      , mp(std::make_unique<Impl>(this))
    {
    }

    /**
    */
    InterpolatorSpiralsYangEtAl::~InterpolatorSpiralsYangEtAl()
    {
    }

    /**
    */
    bool InterpolatorSpiralsYangEtAl::isValid() const
    {
      return true;
    }

    /**
    */
    void InterpolatorSpiralsYangEtAl::setKappa(double kappa)
    {
      mKappa = kappa;
      mp->interpolator.setKappa(kappa);
    }

    /**
    */
    double InterpolatorSpiralsYangEtAl::getKappa() const
    {
      return mKappa;
    }

    /** \brief Interpolation of yang et al

       For each states:
           - use also state before and after
           - use middle point between current state and the one before/after as the (temporal) start and end point of spline
           - use the shorter of the two connections
           - interpolate
           - note: start and end point can't be treated this way
    */
    void InterpolatorSpiralsYangEtAl::interpolate()
    {
      if (mInput.getStates().empty())
      {
        LOG_LINE << __FUNCTION__ << ": Unable to interpolate! The input MukPath has no states!";
        return;
      }
      mInterpolated = true;
      mp->mValidSplinePoints.clear();

      const auto& states = mInput.getStates();
      const size_t N = states.size();
      const size_t M = N >= 3 ? N-2 : 0;
      mp->splines.resize(M);
      mp->splinesFlagsValid.resize(M);
      Vec3d pred, p, next;
      for (int i(1); i<(int)N-1; ++i)
      {
        pred = 0.5*(states[i-1].coords + states[i].coords);
        p = states[i].coords;        
        next = 0.5*(states[i+1].coords + states[i].coords);
        mp->splinesFlagsValid[i-1] = mp->interpolator.interpolate(pred, p, next, mp->splines[i-1]);
      }
    }

    /**
    */
    IInterpolator::result_type InterpolatorSpiralsYangEtAl::getControlPoints() const
    {
      IInterpolator::result_type result;
      result.setRadius(mInput.getRadius());
      result.getStates().reserve(mInput.getStates().size());
      std::copy(mInput.getStates().begin(), mInput.getStates().end(), back_inserter(result.getStates()));
      return result;
    }

    /**
    */
    IInterpolator::result_type InterpolatorSpiralsYangEtAl::getInterpolation(EnInterpolationTypes type) const
    {
      if (!mInterpolated)
        throw MUK_EXCEPTION_SIMPLE("The Interpolator needs te be updated first!");      
      
      std::vector<Vec3d> points;
      const auto& path = mInput.getStates(); // convenience

      if (type != controlPoints)
      {
        points.push_back(path.front().coords);
      }

      switch (type)
      {
        case controlPoints:
        {
          std::transform(path.begin(), path.end(), back_inserter(points), [&] (const MukState& state) { return state.coords; });
          break;
        }
        case samples:
        {
          mp->computeSamples(points);
          points.push_back(path.back().coords);
          break;
        }
        case pointsPerSegment:
        {          
          mp->computeDiscretization(points, mPointsPerSegment);          
          break;
        }
        case resolution:
        {
          mp->computeResolution(points, mResolution);          
          break;          
        }
        default:
        {
          LOG_LINE << __FUNCTION__ << " Interpolation type not implemented!";
        }
      }

      // afterwards linear interpolation between last spline and end point
      if (type == pointsPerSegment || type == resolution)
      {
        bool hasSpline = ! mp->splines.empty();
        const auto& from =  hasSpline ?   mp->splines.back().sample(BezierSpline::E0)   :   path.front().coords;
        const auto& to = path.back().coords;
        MukPath resultLinear = type == pointsPerSegment  ?  computeLinearly(from, to, mPointsPerSegment) : computeLinearly(from, to, mResolution);
        auto first = resultLinear.getStates().begin();
        if (hasSpline)
          ++first;
        auto last  = resultLinear.getStates().end();
        std::transform(first, last, back_inserter(points), [&] (const MukState& state) { return state.coords; });
      }

      // compute tangent
      IInterpolator::result_type result;
      result.setRadius(mInput.getRadius());
      const size_t N = points.size();
      auto& resPath  = result.getStates();
      resPath.reserve(N);
      if (N >= 2)
      {  
        Vec3d tangent = resPath[1].coords-resPath[0].coords;
        tangent.normalize();
        resPath.push_back(MukState(points[0], tangent));
        for (size_t i(1); i<N; ++i)
        {
          tangent = points[i] - points[i-1];
          tangent.normalize();
          resPath.push_back(MukState(points[i], tangent));
        }
      }
      return result;
    }

    /** \brief colors the points depending on the valid flag of the corresponding spline

      computed together with interpolate() because afterwards this is just too complicated
    */
    std::vector<bool> InterpolatorSpiralsYangEtAl::validStates() const
    {      
      return mp->splinesFlagsValid;
    }
    
      
    /**
    */
    void InterpolatorSpiralsYangEtAl::Impl::computeSamples(std::vector<Vec3d>& v)
    { 
      for (size_t i(0); i<splines.size(); ++i)
      {
        std::copy(splines[i].samples.begin(), splines[i].samples.end()-1, std::back_inserter(v));
      }
      if ( ! splines.empty())
        v.push_back(splines.back().sample(7));
    }
    
    /** \brief computes discretization by resolution

      starts with linear interpolation from last point in v to first point of splines
      interpolates then for each spline "inside the spline" and then linearly from end point of current spline to start point of next spline

      does not interpolate from end point of last spline to goal point of MukPath
    */
    void InterpolatorSpiralsYangEtAl::Impl::computeDiscretization(std::vector<Vec3d>& v, size_t mSamples)
    {
      // linear interpolation at the start
      if (splines.empty())
      {
        return;
      }
      const auto& from = v.back();
      const auto& to = splines.front().sample(0);
      auto resultLinear = computeLinearly(from, to, mSamples);
      if (resultLinear.getStates().size() > 2)
      {
        auto omitFirst = resultLinear.getStates().begin()+1;
        auto omitLast = resultLinear.getStates().end()-1;
        std::transform(omitFirst, omitLast, back_inserter(v), [&] (const MukState& state) { return state.coords; });
      }

      // spline interpolation at spline, linear interpolation after spline
      for (int i(0); i<(int)splines.size()-1; ++i)
      {
        const auto& spline = splines[i];
        std::vector<Vec3d> tmp;
        spline.uniformSamples(mSamples, tmp);
        std::copy(tmp.begin(), tmp.end(), back_inserter(v));
        // linear interpolation between splines
        if (spline.sample(BezierSpline::E0) != splines[i+1].sample(BezierSpline::B0))
        {
          const auto& from = splines[i].sample(BezierSpline::E0);
          const auto& to = splines[i+1].sample(BezierSpline::B0);
          auto resultLinear = computeLinearly(from, to, mSamples);
          auto omitLast = resultLinear.getStates().end()-1;
          std::transform(resultLinear.getStates().begin(), omitLast, back_inserter(v), [&] (const MukState& state) { return state.coords; });
        }
      }
      // interpolation at end
      {
        const auto& spline = splines.back();
        std::vector<Vec3d> tmp;
        spline.uniformSamples(mSamples, tmp);
        std::copy(tmp.begin(), tmp.end(), back_inserter(v));
      }
    }

    /** \brief

     Yang's interpolation does not use the input control points, but the points between these.
     Therefore, we also need to interpolate linearly at the beginning and at the end.
     Additionally, to compute the BezierSpline, it uses the smaller one of the two lines P1P2 and P2P3.
     Therefore, we also need to interpolate linearly the longer line from beginning to start of the BezierSpiral.

     implementation as in computeDiscretization with interpolation via resoluion respectively
    */
    void InterpolatorSpiralsYangEtAl::Impl::computeResolution(std::vector<Vec3d>& v, double resolution)
    {
      // linear interpolation at the start
      if ( ! splines.empty())
      {
        const auto& from = v.back();
        const auto& to   = splines.front().sample(0);
        auto resultLinear = computeLinearly(from, to, resolution);
        if (resultLinear.getStates().size() > 2)
        {
          auto omitFirst = resultLinear.getStates().begin()+1;
          auto omitLast = resultLinear.getStates().end()-1;
          std::transform(omitFirst, omitLast, back_inserter(v), [&] (const MukState& state) { return state.coords; });
        }
      }
      // spline interpolation at spline, linear interpolation after spline
      for (int i(0); i<(int)splines.size()-1; ++i)
      {
        const auto& spline = splines[i];
        std::vector<Vec3d> tmp;
        spline.uniformSamples(resolution, tmp);
        std::copy(tmp.begin(), tmp.end(), back_inserter(v));
        // linear interpolation between splines
        if (spline.sample(BezierSpline::E0) != splines[i+1].sample(BezierSpline::B0))
          {
          const auto& from = splines[i].sample(BezierSpline::E0);
          const auto& to   = splines[i+1].sample(BezierSpline::B0);          
          auto resultLinear = computeLinearly(from, to, resolution);
          auto omitLast = resultLinear.getStates().end()-1;
          std::transform(resultLinear.getStates().begin(), omitLast, back_inserter(v), [&] (const MukState& state) { return state.coords;} );
        }
      }
      // interpolation at end
      if ( ! splines.empty())
      {
        const auto& spline = splines.back();
        std::vector<Vec3d> tmp;
        spline.uniformSamples(resolution, tmp);
        std::copy(tmp.begin(), tmp.end(), back_inserter(v));        
      }
    }

  }
}


namespace
{
  /**
  */
  MukPath computeLinearly(const Vec3d& start, const Vec3d& end, size_t discretization)
  {    
    InterpolatorLinear linear;
    MukPath data;
    data.getStates().push_back(MukState(start, Vec3d()));
    data.getStates().push_back(MukState(end, Vec3d()));
    linear.setInput(data);
    linear.setPointsPerSegment(discretization);
    linear.setInterpolationType(IInterpolator::pointsPerSegment);
    linear.interpolate();
    return linear.getInterpolation();
  }
  /**
  */
  MukPath computeLinearly(const Vec3d& start, const Vec3d& end, double resolution)
  {    
    InterpolatorLinear linear;
    MukPath data;
    data.getStates().push_back(MukState(start, Vec3d()));
    data.getStates().push_back(MukState(end, Vec3d()));
    linear.setInput(data);
    linear.setResolution(resolution);
    linear.setInterpolationType(IInterpolator::resolution);
    linear.interpolate();
    return linear.getInterpolation();
  }

}