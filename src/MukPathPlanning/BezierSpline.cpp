#include "private/muk.pch"
#include "BezierSpline.h"

#include "MukCommon/MukException.h"

#include <boost/format.hpp>

namespace
{
  using gris::Vec3d;
  using namespace gris::muk;
  
  Vec3d getBezierPoint(const Vec3d& p0, const Vec3d& p1, const Vec3d& p2, const Vec3d& p3, double t)
  {
    return    1 * p0 * std::pow(1.0-t,3) 
            + 3 * p1 * t*std::pow(1-t, 2) 
            + 3 * p2 * (1-t) * std::pow(t,2) 
            + 1 * p3 * std::pow(t,3);
  }

  Vec3d getDerivativeAtBezierPoint(const Vec3d& p0, const Vec3d& p1, const Vec3d& p2, const Vec3d& p3, double t)
  {
    return     3 * std::pow(1-t,2) * (p1-p0)
            +  6 * (1-t) * t * (p2-p1)
            +  3 * std::pow(t,2) * (p3-p2);      
  }

}

namespace gris
{
  namespace muk
  {
    /**
    */
    BezierCurve::BezierCurve(const Vec3d& p0, const Vec3d& p1, const Vec3d& p2, const Vec3d& p3)
    {
      setPoints(p0, p1, p2, p3);
    }

    /**
    */
    void BezierCurve::setPoints(const Vec3d& p0, const Vec3d& p1, const Vec3d& p2, const Vec3d& p3)
    {
      mPoints[B0] = p0;
      mPoints[B1] = p1;
      mPoints[B2] = p2;
      mPoints[B3] = p3;
    }

    /** \brief no range checking!
    */
    void BezierCurve::setPoint(int i, const Vec3d& p)
    {
      mPoints[i] = p;
    }

    /** \brief returns a sample point
      no range checking!
    */
    const Vec3d& BezierCurve::getPoint(int idx) const
    {
      return mPoints.at(idx);
    }
    
    /** \brief computes length of curve by linear approximation
      
      \param fraction: number of linear segments that approximate the curve
    */
    double BezierCurve::length(size_t fraction) const
    {
      return length(1, fraction);
    }

    /** \brief computes the length of curve until time t by linear approximation 

      \param fraction: number of linear segments that approximate the curve
    */
    double BezierCurve::length(double t, size_t fraction) const
    {
      if (0 == fraction)
        throw MUK_EXCEPTION("invalid parameter", "parameter fraction has to be unequal 0");

      // approx linear
      const double resolution = t / fraction;      
      std::vector<Vec3d> samples;
      for (size_t i(0); i<fraction; ++i)
        samples.push_back(at(i*resolution));
      samples.push_back(mPoints[B3]);
      double length(0);
      for (size_t i(0); i<fraction; ++i)
        length += (samples[i]-samples[i+1]).norm();
      return length;
    }
    
    /** \brief Computes the Point of the curve at time time 
    */
    Vec3d BezierCurve::at(double t) const
    {
      if (t<0 || t>1)
        LOG_LINE << __FUNCTION__ << ": Warning: t (=" << t << ") has to be in [0,1]!";
      t = std::max(0.0, std::min(1.0, t)); // enforce bounds [0,1]

      return getBezierPoint(mPoints[B0], mPoints[B1], mPoints[B2], mPoints[B3], t);    
    }

    /** \brief Computes the derivative of the curve at time t
    */
    Vec3d BezierCurve::derivativeAt(double t) const
    {
      if (t<0 || t>1)
        LOG_LINE << __FUNCTION__ << ": Warning: t (=" << t << ") has to be in [0,1]!";
      t = std::max(0.0, std::min(1.0, t)); // enforce bounds [0,1]
      return getDerivativeAtBezierPoint(mPoints[B0], mPoints[B1], mPoints[B2], mPoints[B3], t);
    }

    /** \brief Computes a approximately uniform sampling of the curve

      \param numel: number of samples to compute
      \param output: times t on which the samples are located
    */
    void BezierCurve::uniformSamples(size_t numel, std::vector<double>& output) const
    {
      const size_t MAX_SIZE = 500;
      if (numel > MAX_SIZE)
        throw MUK_EXCEPTION("maximum size excelled", "implementation of BezierCurve::uniformSamples supports only a maximum of 500 samples"); // to prevent running in excessive calculations

      const size_t N_SAMPLES = 500;
      std::vector<Vec3d> samples(N_SAMPLES);
      const double stepSize = 1.0/N_SAMPLES;
      for (size_t i(0); i<N_SAMPLES; ++i)
      {
        samples[i] = at( i*stepSize );
      }

      const double L          = this->length();
      const double resolution = L/numel;
                  
      output.push_back(0);
      size_t nextIdx  = 1;
      double length   = 0;
      for (size_t i(1); (i<N_SAMPLES) && (numel!=output.size()) ; ++i)
      {
        length += (samples[i]-samples[i-1]).norm();
        if (length > resolution)
        {
          output.push_back(i*stepSize);
          length = 0;
        }
      }
    }
    
    /** 
    */
    double BezierSpline::length(size_t fraction) const
    {
      BezierCurve b(samples[B0], samples[B1], samples[B2], samples[B3]);
      return 2*b.length();
    }

    /**
    */
    double BezierSpline::length(double t, size_t fraction) const
    {
      BezierCurve b(samples[B0], samples[B1], samples[B2], samples[B3]);
      if (t<0.5)
      {        
        t = 2*t;
        return b.length(t);
      }
      else
      {
        t = 2*(t-1); // 2nd spiral starts at the other way
        return 2*b.length() + b.length(1-t);
      }
    }

    /**
    */
    void BezierSpline::uniformSamples(size_t numel, std::vector<Vec3d>& output) const
    {
      const size_t n = numel/2;
      std::vector<double> times;
      BezierCurve b1(samples[B0], samples[B1], samples[B2], samples[B3]);
      BezierCurve b2(samples[E0], samples[E1], samples[E2], samples[B3]);
      b1.uniformSamples(n, times);
      std::transform(times.begin(), times.end(), std::back_inserter(output), [&] (double t)
        {
          return b1.at(t);
        });
      std::transform(times.begin(), times.end(), std::back_inserter(output), [&] (double t)
      {
        return b2.at(1-t);
      });
    }

    /**
    */
    void BezierSpline::uniformSamples(double resolution, std::vector<Vec3d>& output) const
    {
      if (resolution <= 0)
        throw MUK_EXCEPTION("resolution has to be bigger than 0", (boost::format("actual value: %d") % resolution).str().c_str()); 
      const double MIN_RESOLUTION = 0.01; // einfach so
      if (resolution < MIN_RESOLUTION)
        throw MUK_EXCEPTION("resolution has to be bigger than MIN_RESOLUTION", (boost::format("MIN <-> actual value: %d <-> ") % MIN_RESOLUTION % resolution).str().c_str()); 

      BezierCurve b(samples[B0], samples[B1], samples[B2], samples[B3]);
      const double L = 2*b.length();
      const size_t necessarySamples = static_cast<size_t>( L/resolution + 0.5);
      uniformSamples(necessarySamples, output);
    }

    /**
    */
    Vec3d BezierSpline::at(double t) const
    { 
      if (t<0.5)
      {
        BezierCurve b(samples[B0], samples[B1], samples[B2], samples[B3]);
        t = 2*t;
        return b.at(t);
      }
      else
      {        
        BezierCurve b(samples[E0], samples[E1], samples[E2], samples[B3]);
        t = 2*(1-t); // 2nd spiral starts at the other way
        return b.at(t);
      }
    }

    /**
    */
    Vec3d BezierSpline::derivativeAt(double t) const
    {
      if (t<0.5)
      {
        BezierCurve b(samples[B0], samples[B1], samples[B2], samples[B3]);
        t = 2*t;
        return b.derivativeAt(t);
      }
      else
      {        
        BezierCurve b(samples[E0], samples[E1], samples[E2], samples[B3]);
        t = 2*(1-t); // 2nd spiral starts at the other way
        return (-1)* b.derivativeAt(t);
      }
    }

  }
}