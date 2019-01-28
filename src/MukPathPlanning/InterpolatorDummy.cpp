#include "private/muk.pch"
#include "private/InterpolatorFactory.h"

#include "InterpolatorDummy.h"


namespace gris
{
  namespace muk
  {
    REGISTER_INTERPOLATOR(InterpolatorDummy);
    /**
    */
    InterpolatorDummy::InterpolatorDummy()
    {
    }

    /**
    */
    InterpolatorDummy::~InterpolatorDummy()
    {
    }

    /**
    */
    bool InterpolatorDummy::isValid()
    {
      return true;
    }

    /**
    */
    bool InterpolatorDummy::isValid(const MukPath& path)
    {
      return true;
    }
        
    /**
    */
    void InterpolatorDummy::interpolate(const MukPath& path)
    {
    }

    /**
    */
    IInterpolator::result_type InterpolatorDummy::getControlPoints()
    {
      IInterpolator::result_type result;
      result = mInput;
      return result;
    }

    /**
    */
    IInterpolator::result_type InterpolatorDummy::getInterpolation()
    {
      IInterpolator::result_type result;
      result = mInput;
      return result;
    }

    /**
    */
    IInterpolator::result_type InterpolatorDummy::getInterpolation(size_t n)
    {
      IInterpolator::result_type result;
      result = mInput;
      return result;
    }

    /**
    */
    IInterpolator::result_type InterpolatorDummy::getInterpolation(double resolution)
    {
      IInterpolator::result_type result;
      result = mInput;
      return result;
    }

    /**
    */
    IInterpolator::result_type InterpolatorDummy::getInterpolation(const result_type& input, size_t n)
    {
      IInterpolator::result_type result;
      result = mInput;
      return result;
    }

    /**
    */
    IInterpolator::result_type InterpolatorDummy::getInterpolation(const result_type& input, double resolution)
    {
      IInterpolator::result_type result;
      result = mInput;
      return result;
    }


}
}